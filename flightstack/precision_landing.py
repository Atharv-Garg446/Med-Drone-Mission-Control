"""
flightstack/precision_landing.py
================================
Visual precision landing target detection and MAVLink LANDING_TARGET generation.

Capabilities:
  1. ArUco Marker Detection:
     Detects fiducial markers (e.g. DICT_4X4_50) on designated disaster landing pads.
  2. 3D Pose Estimation:
     Estimates translation vector (x, y, z) and orientation in camera optical frame
     using camera intrinsic matrix and marker physical dimensions.
  3. Coordinate Frame Transformation:
     Converts optical frame (X right, Y down, Z forward/depth) into vehicle
     body frame (X forward, Y right, Z down) for downward-pointing gimbal/camera.
  4. MAVLink LANDING_TARGET Message Generation:
     Formats MAVLink message #149 (LANDING_TARGET) accepted by ArduPilot's
     precision landing subsystem (PLND_ENABLED=1, PLND_TYPE=1).
  5. Graceful Degradation:
     If OpenCV is not installed or the target is obscured/absent, cleanly reports
     detected=False with diagnostic reasons, enabling fail-safe hover-and-release
     without blocking the flight controller.

Pure-Python fallback mathematics ensure coordinate transformations and message
structures are fully verifiable in CI environments without OpenCV or GPUs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

try:
    import cv2
    import numpy as np
    HAVE_CV2 = True
except ImportError:
    cv2 = None
    np = None
    HAVE_CV2 = False

# MAVLink frame constants (common.xml)
MAV_FRAME_BODY_NED = 8


@dataclass
class CameraIntrinsics:
    """Downward-facing landing camera calibration parameters."""
    fx: float = 600.0   # Focal length in pixels (X)
    fy: float = 600.0   # Focal length in pixels (Y)
    cx: float = 320.0   # Principal point X (image center)
    cy: float = 240.0   # Principal point Y (image center)
    width: int = 640
    height: int = 480
    distortion: Tuple[float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0)

    @property
    def matrix(self):
        if not HAVE_CV2:
            return None
        return np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

    @property
    def dist_coeffs(self):
        if not HAVE_CV2:
            return None
        return np.array(self.distortion, dtype=np.float64)


@dataclass
class TargetDetection:
    detected: bool
    marker_id: Optional[int] = None
    # Camera optical frame: X=right, Y=down, Z=depth (along camera optical axis)
    pos_camera_m: Optional[Tuple[float, float, float]] = None
    # Vehicle Body-NED frame: X=forward, Y=right, Z=down
    pos_body_m: Optional[Tuple[float, float, float]] = None
    angle_x_rad: float = 0.0    # Azimuth offset (radians)
    angle_y_rad: float = 0.0    # Elevation offset (radians)
    distance_m: float = 0.0     # Euclidean distance to target
    reason: str = ""


def optical_to_body_ned(pos_cam: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Convert 3D point from downward-facing camera frame to vehicle Body-NED frame.

    Camera optical frame:
      +X: Right in image
      +Y: Down in image
      +Z: Forward along lens axis (pointing towards ground)

    Vehicle Body-NED frame:
      +X: Forward (towards nose)
      +Y: Right (towards starboard)
      +Z: Down (towards earth)

    Transformation for camera mounted pointing straight down with top of image aligned forward:
      Body X = -Camera Y
      Body Y = +Camera X
      Body Z = +Camera Z
    """
    xc, yc, zc = pos_cam
    return (-yc, xc, zc)


def angles_from_camera_position(xc: float, yc: float, zc: float) -> Tuple[float, float, float]:
    """Calculate line-of-sight angles and distance from camera coordinates."""
    dist = math.sqrt(xc * xc + yc * yc + zc * zc)
    if zc <= 0:
        return (0.0, 0.0, dist)
    ang_x = math.atan2(xc, zc)
    ang_y = math.atan2(yc, zc)
    return (ang_x, ang_y, dist)


class PrecisionLandingDetector:
    """Processes landing zone imagery and produces target pose estimates."""

    def __init__(
        self,
        intrinsics: Optional[CameraIntrinsics] = None,
        marker_size_m: float = 0.40,     # 40 cm target pad marker
        target_marker_id: int = 0,
        aruco_dict_id: int = 0,          # cv2.aruco.DICT_4X4_50
    ):
        self.intrinsics = intrinsics or CameraIntrinsics()
        self.marker_size_m = marker_size_m
        self.target_marker_id = target_marker_id
        self.aruco_dict_id = aruco_dict_id

    def detect_marker_in_image(self, image_bgr) -> TargetDetection:
        """Detect ArUco marker and estimate 3D position relative to camera and airframe."""
        if not HAVE_CV2:
            return TargetDetection(
                detected=False,
                reason="OpenCV (cv2) not available; visual target detection bypassed.",
            )

        if image_bgr is None or image_bgr.size == 0:
            return TargetDetection(detected=False, reason="Empty or invalid image frame.")

        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        
        # ArUco detection across OpenCV version variations
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_id)
            if hasattr(cv2.aruco, "ArucoDetector"):
                detector = cv2.aruco.ArucoDetector(aruco_dict)
                corners, ids, rejected = detector.detectMarkers(gray)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict)
        except Exception as e:
            return TargetDetection(detected=False, reason=f"ArUco detection error: {e}")

        if ids is None or len(ids) == 0:
            return TargetDetection(detected=False, reason="No ArUco marker visible in landing zone.")

        # Match target ID
        match_idx = None
        for i, mid in enumerate(ids.flatten()):
            if mid == self.target_marker_id:
                match_idx = i
                break
        
        if match_idx is None:
            found_ids = list(ids.flatten())
            return TargetDetection(
                detected=False,
                reason=f"Marker found ({found_ids}) but target ID {self.target_marker_id} not present.",
            )

        marker_corners = corners[match_idx]

        # 3D pose estimation via solvePnP
        # Define 3D object points for planar square marker centered at origin
        hs = self.marker_size_m / 2.0
        obj_pts = np.array([
            [-hs,  hs, 0.0],
            [ hs,  hs, 0.0],
            [ hs, -hs, 0.0],
            [-hs, -hs, 0.0],
        ], dtype=np.float64)

        success, rvec, tvec = cv2.solvePnP(
            obj_pts, marker_corners[0],
            self.intrinsics.matrix, self.intrinsics.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE") else cv2.SOLVEPNP_ITERATIVE,
        )

        if not success:
            return TargetDetection(detected=False, reason="Pose estimation (solvePnP) failed.")

        xc, yc, zc = float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])
        ang_x, ang_y, dist = angles_from_camera_position(xc, yc, zc)
        pos_body = optical_to_body_ned((xc, yc, zc))

        return TargetDetection(
            detected=True,
            marker_id=self.target_marker_id,
            pos_camera_m=(round(xc, 4), round(yc, 4), round(zc, 4)),
            pos_body_m=(round(pos_body[0], 4), round(pos_body[1], 4), round(pos_body[2], 4)),
            angle_x_rad=round(ang_x, 4),
            angle_y_rad=round(ang_y, 4),
            distance_m=round(dist, 4),
            reason="Target locked",
        )


def build_landing_target_mavlink_payload(
    detection: TargetDetection,
    time_usec: int = 0,
    target_num: int = 0,
    frame: int = MAV_FRAME_BODY_NED,
) -> Optional[dict]:
    """Format dictionary representation of MAVLink LANDING_TARGET (message #149)."""
    if not detection.detected or detection.pos_body_m is None:
        return None

    bx, by, bz = detection.pos_body_m
    return {
        "message_type": "LANDING_TARGET",
        "time_usec": time_usec,
        "target_num": target_num,
        "frame": frame,
        "angle_x": detection.angle_x_rad,
        "angle_y": detection.angle_y_rad,
        "distance": detection.distance_m,
        "size_x": 0.40,
        "size_y": 0.40,
        "x": bx,
        "y": by,
        "z": bz,
        "q": [1.0, 0.0, 0.0, 0.0],
        "type": 0,    # LANDING_TARGET_TYPE_LIGHT_BEACON / VISION_FIDUCIAL
        "position_valid": 1,
    }


def generate_synthetic_landing_pad(
    marker_id: int = 0,
    marker_size_px: int = 200,
    pad_size_px: int = 400,
) -> Optional[object]:
    """Generate a synthetic landing pad image (BGR) with an ArUco fiducial marker."""
    if not HAVE_CV2:
        return None
    # White canvas representing prepared landing zone pad
    img = np.ones((pad_size_px, pad_size_px, 3), dtype=np.uint8) * 240
    # Red circle border identifying medical helicopter / drone pad
    cv2.circle(img, (pad_size_px // 2, pad_size_px // 2), pad_size_px // 2 - 10, (0, 0, 200), 4)

    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)
        elif hasattr(cv2.aruco, "drawMarker"):
            marker_img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size_px)
        else:
            return None

        marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)
        offset = (pad_size_px - marker_size_px) // 2
        img[offset:offset + marker_size_px, offset:offset + marker_size_px] = marker_bgr
        return img
    except Exception:
        return None


class PrecisionLandingController:
    """Orchestrates image acquisition, target detection, MAVLink transmission, and failsafes."""

    def __init__(
        self,
        detector: Optional[PrecisionLandingDetector] = None,
        link: Optional[object] = None,
        log: Optional[object] = None,
        update_rate_hz: float = 10.0,
        stale_timeout_s: float = 1.0,
        abort_alt_m: float = 5.0,
    ):
        self.detector = detector or PrecisionLandingDetector()
        self.link = link
        self.log = log
        self.update_interval_s = 1.0 / max(update_rate_hz, 1.0)
        self.stale_timeout_s = stale_timeout_s
        self.abort_alt_m = abort_alt_m
        self.last_target: Optional[TargetDetection] = None
        self.last_detection_time_s: float = 0.0
        self.last_send_time_s: float = 0.0
        self.targets_sent_count: int = 0
        self.is_locked: bool = False

    def process_frame(self, frame_bgr, timestamp_s: float, current_alt_m: float = 15.0) -> TargetDetection:
        """Process one video frame, send LANDING_TARGET to MAVLink if locked, handle loss."""
        det = self.detector.detect_marker_in_image(frame_bgr)
        now = timestamp_s

        if det.detected:
            self.last_target = det
            self.last_detection_time_s = now
            if not self.is_locked:
                self.is_locked = True
                if self.log:
                    self.log.log("precision_landing_target_locked",
                                 marker_id=det.marker_id,
                                 distance_m=det.distance_m,
                                 pos_body_m=det.pos_body_m)

            # Rate-controlled transmission to MAVLink
            if now - self.last_send_time_s >= self.update_interval_s:
                self.last_send_time_s = now
                self.send_to_mavlink(det, int(now * 1e6))
        else:
            # Check for target loss
            if self.is_locked and (now - self.last_detection_time_s > self.stale_timeout_s):
                self.is_locked = False
                if self.log:
                    self.log.log("precision_landing_target_lost",
                                 reason=det.reason,
                                 time_since_last_s=round(now - self.last_detection_time_s, 2),
                                 current_alt_m=current_alt_m)
                # Landing abort if too low without target lock
                if current_alt_m < self.abort_alt_m and self.link:
                    if hasattr(self.link, "set_mode"):
                        self.link.set_mode("LOITER")
                    if self.log:
                        self.log.log("precision_landing_aborted",
                                     action="LOITER",
                                     reason=f"Target lost below abort altitude ({current_alt_m:.1f}m < {self.abort_alt_m:.1f}m)")

        return det

    def send_to_mavlink(self, det: TargetDetection, time_usec: int = 0) -> bool:
        """Transmit LANDING_TARGET message over MAVLink connection."""
        if not det.detected or det.pos_body_m is None or self.link is None:
            return False

        bx, by, bz = det.pos_body_m
        conn = getattr(self.link, "conn", None)
        if conn is None or not hasattr(conn, "mav"):
            return False

        try:
            lock = getattr(self.link, "_send_lock", None)
            if lock is not None:
                with lock:
                    conn.mav.landing_target_send(
                        time_usec,
                        0,                      # target_num
                        MAV_FRAME_BODY_NED,     # frame
                        det.angle_x_rad,        # angle_x
                        det.angle_y_rad,        # angle_y
                        det.distance_m,         # distance
                        0.40,                   # size_x
                        0.40,                   # size_y
                        bx, by, bz,             # x, y, z
                        [1.0, 0.0, 0.0, 0.0],   # q
                        0,                      # type (vision fiducial)
                        1                       # position_valid
                    )
            else:
                conn.mav.landing_target_send(
                    time_usec, 0, MAV_FRAME_BODY_NED,
                    det.angle_x_rad, det.angle_y_rad, det.distance_m,
                    0.40, 0.40, bx, by, bz,
                    [1.0, 0.0, 0.0, 0.0], 0, 1
                )
            self.targets_sent_count += 1
            if self.log:
                self.log.log("landing_target_sent",
                             count=self.targets_sent_count,
                             pos_body_m=det.pos_body_m,
                             distance_m=det.distance_m)
            return True
        except Exception as e:
            if self.log:
                self.log.log("landing_target_send_error", error=str(e))
            return False


