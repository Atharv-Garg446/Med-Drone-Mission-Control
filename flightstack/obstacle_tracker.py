"""
flightstack/obstacle_tracker.py
===============================
Dynamic obstacle tracking, Closest Point of Approach (CPA) conflict prediction,
and reactive autopilot response.

Clear Separation of Responsibilities:
  1. Global Strategic Planning:
     Static polygon no-fly zones (airports, high-security facilities) are solved
     via 8-connected grid A* in nofly_astar.py before flight.
  2. Dynamic Perception & Tracking:
     Tracks moving 3D obstacles (non-cooperative UAVs, cranes, aerial hazards)
     using verified spatial position (x, y, z) and velocity (vx, vy, vz).
     Does not infer distance from 2D bounding boxes alone.
  3. Tactical Conflict Detection:
     Calculates Time-to-Closest-Point-of-Approach (t_cpa) and distance at CPA (d_cpa).
     Predicts flight corridor infringements within a configurable horizon.
  4. Reactive Autopilot Response:
     - Immediate Tactical Hold: Commands autopilot LOITER/BRAKE via MAVLink when
       an imminent collision risk (e.g. within 10s and <30m) is detected.
     - Strategic Reroute: If an obstacle remains stationary or persistently blocks
       the corridor, creates an inflated exclusion zone and invokes dynamic A*
       replanning from the autopilot's current position estimate.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class Vector3D:
    x: float  # North / meters
    y: float  # East / meters
    z: float  # Down / meters

    def norm(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def dot(self, other: "Vector3D") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def sub(self, other: "Vector3D") -> "Vector3D":
        return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def add(self, other: "Vector3D") -> "Vector3D":
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def scale(self, factor: float) -> "Vector3D":
        return Vector3D(self.x * factor, self.y * factor, self.z * factor)


@dataclass
class TrackedObstacle:
    obstacle_id: str
    position_m: Vector3D     # Local NED relative to origin/home
    velocity_ms: Vector3D    # Velocity in NED (m/s)
    confidence: float = 1.0
    radius_m: float = 5.0    # Physical bounding sphere radius
    last_update_s: float = field(default_factory=time.monotonic)
    is_persistent: bool = False  # If True, not purged on high staleness (e.g. stationary crane/tower)

    def predict_position(self, dt_seconds: float) -> Vector3D:
        return self.position_m.add(self.velocity_ms.scale(dt_seconds))


@dataclass
class ConflictAlert:
    has_conflict: bool
    obstacle_id: str
    t_cpa_s: float
    d_cpa_m: float
    current_distance_m: float
    action_required: str  # "NONE" | "HOLD" | "REROUTE"
    detail: str = ""


class ObstacleTracker:
    """Manages active dynamic obstacles and predicts trajectory conflicts."""

    def __init__(
        self,
        safety_margin_m: float = 30.0,
        time_horizon_s: float = 12.0,
        staleness_limit_s: float = 2.0,
    ):
        self.safety_margin_m = safety_margin_m
        self.time_horizon_s = time_horizon_s
        self.staleness_limit_s = staleness_limit_s
        self.tracks: Dict[str, TrackedObstacle] = {}

    def update_track(
        self,
        obstacle_id: str,
        pos_ned_m: Tuple[float, float, float],
        vel_ned_ms: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        radius_m: float = 5.0,
        confidence: float = 1.0,
        timestamp_s: Optional[float] = None,
        is_persistent: bool = False,
    ) -> TrackedObstacle:
        """Update or register a 3D tracked obstacle with spatial position and velocity."""
        now = timestamp_s if timestamp_s is not None else time.monotonic()
        track = TrackedObstacle(
            obstacle_id=obstacle_id,
            position_m=Vector3D(*pos_ned_m),
            velocity_ms=Vector3D(*vel_ned_ms),
            confidence=confidence,
            radius_m=radius_m,
            last_update_s=now,
            is_persistent=is_persistent,
        )
        self.tracks[obstacle_id] = track
        return track

    def update_track_geo(
        self,
        obstacle_id: str,
        lat: float,
        lon: float,
        alt_rel_m: float,
        origin_lat: float,
        origin_lon: float,
        vel_ned_ms: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        radius_m: float = 10.0,
        confidence: float = 1.0,
        timestamp_s: Optional[float] = None,
        is_persistent: bool = False,
    ) -> TrackedObstacle:
        """Update or register an obstacle given geographic (lat, lon, alt_rel) and home origin."""
        pos_ned = geo_to_ned(lat, lon, alt_rel_m, origin_lat, origin_lon)
        return self.update_track(
            obstacle_id=obstacle_id,
            pos_ned_m=pos_ned,
            vel_ned_ms=vel_ned_ms,
            radius_m=radius_m,
            confidence=confidence,
            timestamp_s=timestamp_s,
            is_persistent=is_persistent,
        )

    def update_from_spatial_observation(self, obs: object) -> Optional[TrackedObstacle]:
        """Ingest a SpatialObservation from the perception pipeline.

        Enforces:
          - Rejects observations without metric depth (refuses to fabricate 3D avoidance from 2D boxes).
          - Enforces minimum confidence threshold (0.25).
        """
        if not getattr(obs, "depth_available", False):
            return None
        pos = getattr(obs, "position_ned", None)
        if pos is None:
            return None
        if getattr(obs, "confidence", 1.0) < 0.25:
            return None

        vel = getattr(obs, "velocity_ned", None)
        vel_tuple = (vel.x, vel.y, vel.z) if vel else (0.0, 0.0, 0.0)

        return self.update_track(
            obstacle_id=getattr(obs, "obstacle_id", "OBS_UNKNOWN"),
            pos_ned_m=(pos.x, pos.y, pos.z),
            vel_ned_ms=vel_tuple,
            radius_m=getattr(obs, "radius_m", 10.0),
            confidence=getattr(obs, "confidence", 1.0),
            timestamp_s=getattr(obs, "timestamp_s", None),
            is_persistent=False,
        )

    def purge_stale_tracks(self, now: Optional[float] = None) -> int:
        cur = now if now is not None else time.monotonic()
        stale_keys = [
            k for k, tr in self.tracks.items()
            if not tr.is_persistent and (cur - tr.last_update_s > self.staleness_limit_s)
        ]
        for k in stale_keys:
            del self.tracks[k]
        return len(stale_keys)


    def evaluate_conflict(
        self,
        drone_pos_ned: Tuple[float, float, float],
        drone_vel_ned: Tuple[float, float, float],
        now: Optional[float] = None,
    ) -> List[ConflictAlert]:
        """Compute Closest Point of Approach (CPA) against all fresh obstacle tracks.

        Returns list of ConflictAlerts sorted by earliest time to conflict.
        """
        p_drone = Vector3D(*drone_pos_ned)
        v_drone = Vector3D(*drone_vel_ned)
        alerts: List[ConflictAlert] = []

        cur = now if now is not None else time.monotonic()

        for obs_id, track in self.tracks.items():
            if cur - track.last_update_s > self.staleness_limit_s:
                continue

            r_rel = track.position_m.sub(p_drone)
            v_rel = track.velocity_ms.sub(v_drone)

            cur_dist = r_rel.norm()
            rel_speed_sq = v_rel.dot(v_rel)

            total_safety = self.safety_margin_m + track.radius_m

            # If objects are stationary relative to each other
            if rel_speed_sq < 1e-4:
                t_cpa = 0.0
                d_cpa = cur_dist
            else:
                # t_cpa = - (r . v) / |v|^2
                t_cpa = -r_rel.dot(v_rel) / rel_speed_sq
                if t_cpa < 0.0:
                    # Moving away from each other; closest point is current time
                    t_cpa = 0.0
                    d_cpa = cur_dist
                else:
                    # Closest distance in future
                    pos_cpa = r_rel.add(v_rel.scale(t_cpa))
                    d_cpa = pos_cpa.norm()

            is_conflict = (0.0 <= t_cpa <= self.time_horizon_s) and (d_cpa < total_safety)

            action = "NONE"
            detail = f"Track {obs_id}: d_cur={cur_dist:.1f}m, d_cpa={d_cpa:.1f}m at t={t_cpa:.1f}s"
            if is_conflict:
                if t_cpa < 5.0 or cur_dist < total_safety:
                    action = "HOLD"
                    detail += " [CRITICAL: Immediate LOITER/Hold commanded]"
                else:
                    action = "REROUTE"
                    detail += " [WARNING: Corridor conflict projected]"

            alerts.append(ConflictAlert(
                has_conflict=is_conflict,
                obstacle_id=obs_id,
                t_cpa_s=round(t_cpa, 2),
                d_cpa_m=round(d_cpa, 2),
                current_distance_m=round(cur_dist, 2),
                action_required=action,
                detail=detail,
            ))

        alerts.sort(key=lambda a: (not a.has_conflict, a.t_cpa_s))
        return alerts


def obstacle_to_exclusion_polygon(
    lat: float,
    lon: float,
    radius_m: float = 50.0,
    num_vertices: int = 8,
) -> List[Tuple[float, float]]:
    """Convert a detected stationary obstacle position into a polygon for A*."""
    import math
    vertices = []
    dlat_per_m = 1.0 / 111139.0
    dlon_per_m = 1.0 / (111139.0 * math.cos(math.radians(lat)))

    for i in range(num_vertices):
        angle = 2.0 * math.pi * (i / num_vertices)
        v_lat = lat + (radius_m * math.sin(angle)) * dlat_per_m
        v_lon = lon + (radius_m * math.cos(angle)) * dlon_per_m
        vertices.append((round(v_lat, 7), round(v_lon, 7)))
    return vertices


def geo_to_ned(
    lat: float,
    lon: float,
    alt_rel_m: float,
    origin_lat: float,
    origin_lon: float,
) -> Tuple[float, float, float]:
    """Convert (lat, lon, alt_rel_m) to local NED meters relative to (origin_lat, origin_lon)."""
    dlat = lat - origin_lat
    dlon = lon - origin_lon
    north = dlat * 111139.0
    east = dlon * (111139.0 * math.cos(math.radians(origin_lat)))
    down = -alt_rel_m
    return (round(north, 3), round(east, 3), round(down, 3))


def ned_to_geo(
    north_m: float,
    east_m: float,
    down_m: float,
    origin_lat: float,
    origin_lon: float,
) -> Tuple[float, float, float]:
    """Convert local NED meters relative to origin back to (lat, lon, alt_rel_m)."""
    lat = origin_lat + (north_m / 111139.0)
    lon = origin_lon + (east_m / (111139.0 * math.cos(math.radians(origin_lat))))
    alt_rel = -down_m
    return (round(lat, 7), round(lon, 7), round(alt_rel, 3))

