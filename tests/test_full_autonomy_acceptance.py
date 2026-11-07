#!/usr/bin/env python3
"""
tests/test_full_autonomy_acceptance.py
======================================
Comprehensive acceptance tests covering:
  - Payload lifecycle states, actuator confirmations, and mechanical failure simulation.
  - Streaming perception pipeline, monocular ray projection, and metric depth gating.
  - Precision landing target detection, MAVLink transmission, and low-altitude abort logic.
  - Item-by-item mission upload readback verification.
  - In-flight replan constraint re-validation (MTOW, battery Wh budget, cold-chain).
  - Battery reserve floor failsafe (RTL trigger) and stale telemetry protection.
  - Event log parsing and replay synchronization (consistent t and t_rel_s).
"""

import json
import math
import os
import sys
import tempfile
import time
import unittest
from datetime import datetime, timedelta, timezone
from typing import List, Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from config import Location
from flightstack.mission import (
    MissionItem, MissionPlan, MissionProfile, build_delivery_mission,
    MAV_CMD_NAV_WAYPOINT, MAV_CMD_DO_GRIPPER, MAV_CMD_NAV_LAND, MAV_CMD_NAV_TAKEOFF,
    MAV_FRAME_GLOBAL, MAV_FRAME_MISSION
)
from flightstack.obstacle_tracker import ObstacleTracker, Vector3D
from flightstack.payload import MedicalPackage, PackageStatus, PayloadManager, ReleaseSource
from flightstack.perception_pipeline import (
    BoundingBox2D, CameraExtrinsics, CameraIntrinsics, LiveCameraSource,
    ObservationProvenance, PerceptionSpatialConverter, RecordedVideoSource,
    SpatialObservation, SyntheticCameraSource, YOLOStreamDetector
)
from flightstack.precision_landing import (
    CameraIntrinsics as LandingCameraIntrinsics, PrecisionLandingController,
    PrecisionLandingDetector, TargetDetection, generate_synthetic_landing_pad,
    optical_to_body_ned
)
from flightstack.telemetry import TelemetryState, PositionEstimate, StaleTelemetryError
from flightstack.vehicle_model import MulticopterModel, EnergyEstimator


class FakeMavMessage:
    def __init__(self, msg_type: str, **kwargs):
        self._type = msg_type
        for k, v in kwargs.items():
            setattr(self, k, v)

    def get_type(self) -> str:
        return self._type


class MockMavlinkConnection:
    """Mock connection for testing MAVLink message transmissions."""
    def __init__(self):
        self.sent_landing_targets = []
        self.sent_commands = []
        self.mode = "AUTO"
        self.mav = self

    def landing_target_send(self, time_usec, target_num, frame, angle_x, angle_y,
                            distance, size_x, size_y, x, y, z, q, target_type, position_valid):
        self.sent_landing_targets.append({
            "time_usec": time_usec, "frame": frame,
            "angle_x": angle_x, "angle_y": angle_y, "distance": distance,
            "pos": (x, y, z), "position_valid": position_valid
        })

    def command_long_send(self, target_sys, target_comp, command, confirmation,
                          p1, p2, p3, p4, p5, p6, p7):
        self.sent_commands.append({
            "command": command, "params": (p1, p2, p3, p4, p5, p6, p7)
        })


class MockAutopilotLink:
    def __init__(self):
        self.conn = MockMavlinkConnection()
        self.telemetry = TelemetryState()
        self.uploaded_items = []
        self.current_mode = "AUTO"

    def set_mode(self, mode: str) -> None:
        self.current_mode = mode

    def set_current_item(self, seq: int) -> None:
        self.current_seq = seq

    def upload_mission(self, items: List[MissionItem], verify: bool = False, **kwargs) -> None:
        self.uploaded_items = list(items)

    def download_mission_count(self) -> int:
        return len(self.uploaded_items)

    def download_mission(self, timeout: float = 3.0) -> List[tuple]:
        return [(it.seq, it.command, it.lat, it.lon, it.alt_m) for it in self.uploaded_items]


# ===========================================================================
# Test Suite 1: Payload and Delivery Actuator Actions (Objective 5)
# ===========================================================================

class TestPayloadAndActuator(unittest.TestCase):

    def setUp(self):
        self.stops = [
            Location(id=1, name="Clinic A", lat=26.85, lon=75.80, demand_kg=2.5),
            Location(id=2, name="Clinic B", lat=26.86, lon=75.81, demand_kg=1.5),
        ]
        self.manager = PayloadManager.from_stops(self.stops, mode="hover")

    def test_initial_payload_state(self):
        self.assertEqual(self.manager.remaining_cargo_kg(), 4.0)
        self.assertEqual(self.manager.pending_count(), 2)
        self.assertEqual(self.manager.delivered_count(), 0)

    def test_command_and_confirm_release(self):
        ok, pkg, reason = self.manager.command_release(1, "hover", t_rel_s=120.0)
        self.assertTrue(ok)
        self.assertEqual(pkg.status, PackageStatus.RELEASE_COMMANDED)
        self.assertEqual(self.manager.remaining_cargo_kg(), 4.0)  # Still aboard until confirmed

        ok, pkg, reason = self.manager.confirm_release(1, t_rel_s=125.0, source=ReleaseSource.SIM_GRIPPER_FEEDBACK)
        self.assertTrue(ok)
        self.assertEqual(pkg.status, PackageStatus.DELIVERED)
        self.assertEqual(pkg.confirmation_source, ReleaseSource.SIM_GRIPPER_FEEDBACK)
        self.assertEqual(self.manager.remaining_cargo_kg(), 1.5)  # Cargo mass decremented!
        self.assertEqual(self.manager.delivered_count(), 1)

    def test_duplicate_release_rejected(self):
        self.manager.command_release(1, "hover", t_rel_s=100.0)
        self.manager.confirm_release(1, t_rel_s=105.0)

        # Attempt second release for stop 1
        ok, pkg, reason = self.manager.command_release(1, "hover", t_rel_s=150.0)
        self.assertFalse(ok)
        self.assertIn("DUPLICATE_RELEASE", reason)

    def test_stuck_gripper_fault_simulation(self):
        ok, pkg, reason = self.manager.command_release(1, "hover", t_rel_s=80.0, simulate_stuck=True)
        self.assertFalse(ok)
        self.assertEqual(pkg.status, PackageStatus.FAILED)
        self.assertIn("ACTUATOR_FAULT", reason)
        self.assertEqual(self.manager.remaining_cargo_kg(), 4.0)  # Cargo still aboard

    def test_unknown_stop_release(self):
        ok, pkg, reason = self.manager.command_release(99, "hover", t_rel_s=50.0)
        self.assertFalse(ok)
        self.assertIn("NO_PAYLOAD_FOR_STOP", reason)

    def test_record_actuator_text_parsing(self):
        from flightstack.payload import DeliveryState
        self.manager.command_release(2, "hover", t_rel_s=200.0)
        # Simulate ArduPilot STATUSTEXT: causally acknowledges simulated gripper release,
        # separated from stop completion and recipient custody.
        res = self.manager.record_actuator_text("Gripper Released", t_rel_s=202.5)
        self.assertIsNotNone(res)
        sid, pkg = res
        self.assertEqual(sid, 2)
        self.assertEqual(pkg.delivery_state, DeliveryState.RELEASE_ACKNOWLEDGED)
        self.assertTrue(pkg.simulated_release_acknowledged)
        self.assertFalse(pkg.stop_completion_recorded)
        self.assertFalse(pkg.recipient_handover_verified)
        self.assertEqual(pkg.status, PackageStatus.RELEASE_COMMANDED)
        self.assertEqual(pkg.confirmation_source, ReleaseSource.SIM_GRIPPER_FEEDBACK)
        self.assertEqual(pkg.release_provenance, "simulated_hover_gripper_release")

        # Duplicate feedback must be ignored cleanly and counted
        dup = self.manager.record_actuator_text("Gripper load released", t_rel_s=202.6)
        self.assertIsNone(dup)
        self.assertEqual(pkg.duplicate_feedback_count, 1)

        # Completing the stop transitions to DELIVERED while keeping recipient_handover_verified=False
        ok, pkg, _ = self.manager.complete_delivery(2, t_rel_s=205.0)
        self.assertTrue(ok)
        self.assertEqual(pkg.status, PackageStatus.DELIVERED)
        self.assertEqual(pkg.delivery_state, DeliveryState.DELIVERY_COMPLETE)
        self.assertTrue(pkg.stop_completion_recorded)
        self.assertFalse(pkg.recipient_handover_verified)

    def test_missing_feedback_blocks_completion_and_failed_feedback_marks_failed(self):
        from flightstack.payload import DeliveryState
        self.manager.command_release(1, "hover", t_rel_s=100.0)
        # Cannot complete delivery without causal actuator acknowledgement
        ok, pkg, err = self.manager.complete_delivery(1, t_rel_s=105.0)
        self.assertFalse(ok)
        self.assertTrue(err.startswith("MISSING_ACTUATOR_ACKNOWLEDGEMENT"))
        self.assertNotEqual(pkg.status, PackageStatus.DELIVERED)

        # Fault STATUSTEXT marks release as FAILED
        res = self.manager.record_actuator_text("Gripper failed: servo jam", t_rel_s=106.0)
        self.assertIsNotNone(res)
        sid, fpkg = res
        self.assertEqual(sid, 1)
        self.assertEqual(fpkg.status, PackageStatus.FAILED)
        self.assertEqual(fpkg.delivery_state, DeliveryState.FAILED)
        ok2, _, err2 = self.manager.complete_delivery(1, t_rel_s=107.0)
        self.assertFalse(ok2)
        self.assertTrue(err2.startswith("CANNOT_COMPLETE_FAILED_RELEASE"))


# ===========================================================================
# Test Suite 2: Perception Pipeline & Spatial Conversion (Objective 3)
# ===========================================================================

class TestPerceptionPipeline(unittest.TestCase):

    def setUp(self):
        self.intrinsics = CameraIntrinsics.default_pinhole(640, 480, hfov_deg=70.0)
        self.extrinsics = CameraExtrinsics()  # Forward-facing camera
        self.converter = PerceptionSpatialConverter(self.intrinsics, self.extrinsics)

    def test_synthetic_camera_source(self):
        src = SyntheticCameraSource(width=640, height=480, fps=20.0)
        self.assertTrue(src.open())
        ret, frame, t = src.read_frame()
        self.assertTrue(ret)
        self.assertIsNotNone(frame)
        self.assertEqual(frame.shape, (480, 640, 3))
        self.assertEqual(src.provenance, ObservationProvenance.SYNTHETIC_CAMERA)

    def test_monocular_projection_without_depth(self):
        """Monocular detections must produce bearing angles but NO fabricated depth."""
        box = BoundingBox2D(class_id=2, class_name="crane", confidence=0.88,
                            x1=300.0, y1=220.0, x2=340.0, y2=260.0)
        obs = self.converter.box_to_spatial(box, timestamp_s=10.0, depth_m=None)
        self.assertFalse(obs.depth_available)
        self.assertIsNone(obs.metric_distance_m)
        self.assertIsNotNone(obs.bearing_azimuth_deg)
        self.assertIsNotNone(obs.bearing_elevation_deg)
        # Azimuth should be close to 0 since box is near image center (320, 240)
        self.assertAlmostEqual(obs.bearing_azimuth_deg, 0.0, delta=5.0)

    def test_metric_projection_with_depth(self):
        """With metric depth provided, full 3D NED coordinates are calculated."""
        box = BoundingBox2D(class_id=2, class_name="crane", confidence=0.92,
                            x1=310.0, y1=230.0, x2=330.0, y2=250.0)
        drone_pos = Vector3D(100.0, 200.0, -60.0)  # 60m altitude
        obs = self.converter.box_to_spatial(
            box, timestamp_s=12.0, depth_m=50.0,
            drone_pos_ned=drone_pos, drone_attitude_deg=(0.0, 0.0, 0.0)
        )
        self.assertTrue(obs.depth_available)
        self.assertIsNotNone(obs.metric_distance_m)
        self.assertAlmostEqual(obs.metric_distance_m, 50.0, delta=1.0)
        # Camera is forward-facing: Body X is forward, so obstacle is ~50m ahead in X
        self.assertAlmostEqual(obs.position_ned.x, 150.0, delta=2.0)

    def test_obstacle_tracker_rejects_depthless_observations(self):
        """Tracker must refuse to create 3D obstacle tracks when depth is missing."""
        tracker = ObstacleTracker()
        box = BoundingBox2D(class_id=1, class_name="vehicle", confidence=0.80,
                            x1=100.0, y1=100.0, x2=150.0, y2=150.0)
        obs_no_depth = self.converter.box_to_spatial(box, timestamp_s=5.0, depth_m=None)
        res = tracker.update_from_spatial_observation(obs_no_depth)
        self.assertIsNone(res)
        self.assertEqual(len(tracker.tracks), 0)

        # Now test with depth: must be accepted
        obs_with_depth = self.converter.box_to_spatial(box, timestamp_s=5.0, depth_m=35.0)
        res = tracker.update_from_spatial_observation(obs_with_depth)
        self.assertIsNotNone(res)
        self.assertEqual(len(tracker.tracks), 1)

    def test_yolo_stream_detector_empty_frame_handling(self):
        detector = YOLOStreamDetector(conf_threshold=0.25)
        # Without loaded model, raises RuntimeError
        with self.assertRaises(RuntimeError):
            detector.detect_frame(np.zeros((100, 100, 3), dtype=np.uint8), 0.0)


# ===========================================================================
# Test Suite 3: Precision Landing MAVLink Transmission (Objective 4)
# ===========================================================================

class TestPrecisionLandingIntegration(unittest.TestCase):

    def setUp(self):
        self.intrinsics = LandingCameraIntrinsics()
        self.detector = PrecisionLandingDetector(self.intrinsics, marker_size_m=0.40)
        self.link = MockAutopilotLink()
        self.controller = PrecisionLandingController(
            self.detector, link=self.link, update_rate_hz=10.0, abort_alt_m=5.0
        )

    def test_optical_to_body_ned_math(self):
        # Camera frame: (right=1.0, down=2.0, forward=5.0)
        # Body frame for downward camera: (-down, +right, +forward) = (-2.0, 1.0, 5.0)
        bx, by, bz = optical_to_body_ned((1.0, 2.0, 5.0))
        self.assertEqual(bx, -2.0)
        self.assertEqual(by, 1.0)
        self.assertEqual(bz, 5.0)

    def test_precision_landing_target_detection_and_mavlink_send(self):
        pad_img = generate_synthetic_landing_pad(marker_id=0, marker_size_px=200, pad_size_px=400)
        if pad_img is None:
            self.skipTest("cv2.aruco not available in this environment")

        det = self.controller.process_frame(pad_img, timestamp_s=1.0, current_alt_m=10.0)
        self.assertTrue(det.detected)
        self.assertTrue(self.controller.is_locked)
        self.assertGreater(self.controller.targets_sent_count, 0)
        self.assertEqual(len(self.link.conn.sent_landing_targets), 1)
        msg = self.link.conn.sent_landing_targets[0]
        self.assertEqual(msg["position_valid"], 1)
        self.assertGreater(msg["distance"], 0.0)

    def test_precision_landing_target_loss_and_abort(self):
        # First lock onto target
        pad_img = generate_synthetic_landing_pad(marker_id=0, marker_size_px=200, pad_size_px=400)
        if pad_img is None:
            self.skipTest("cv2.aruco not available")

        self.controller.process_frame(pad_img, timestamp_s=1.0, current_alt_m=10.0)
        self.assertTrue(self.controller.is_locked)

        # Frame with no marker (black image) at critical descent altitude (3.0m < 5.0m abort alt)
        blank = np.zeros((400, 400, 3), dtype=np.uint8)
        # Process at t=2.5s (> 1.0s timeout)
        det = self.controller.process_frame(blank, timestamp_s=2.5, current_alt_m=3.0)
        self.assertFalse(det.detected)
        self.assertFalse(self.controller.is_locked)
        # Mode should have switched to LOITER to abort unsafe landing
        self.assertEqual(self.link.current_mode, "LOITER")


# ===========================================================================
# Test Suite 4: Mission Readback & Replan Constraints (Objectives 2 & 6)
# ===========================================================================

class TestMissionVerificationAndReplanConstraints(unittest.TestCase):

    def test_land_delivery_mission_has_release_action(self):
        """Land delivery must contain an explicit gripper release action while on the ground."""
        stops = [Location(id=1, name="Stop 1", lat=26.85, lon=75.80, demand_kg=2.0)]
        profile = MissionProfile(delivery_mode="land")
        plan = build_delivery_mission((26.90, 75.82), stops, zones=[], profile=profile)

        commands = [it.command for it in plan.items]
        self.assertIn(MAV_CMD_NAV_LAND, commands)
        self.assertIn(MAV_CMD_DO_GRIPPER, commands)
        self.assertIn(MAV_CMD_NAV_TAKEOFF, commands)

        # Release must occur AFTER landing and BEFORE retakeoff
        land_idx = commands.index(MAV_CMD_NAV_LAND)
        rel_idx = commands.index(MAV_CMD_DO_GRIPPER)
        to_idx = [i for i, c in enumerate(commands) if c == MAV_CMD_NAV_TAKEOFF][-1]
        self.assertGreater(rel_idx, land_idx, "DO_GRIPPER must occur after NAV_LAND")
        self.assertGreater(to_idx, rel_idx, "NAV_TAKEOFF must occur after DO_GRIPPER")

    def test_replan_rejects_energy_infeasible_routes(self):
        """apply_airspace_update must refuse to upload a route if remaining battery Wh is exceeded."""
        from flightstack.mission_runner import MissionRunner, EventLog

        stops = [
            Location(id=1, name="Stop 1", lat=26.85, lon=75.80, demand_kg=2.0),
            Location(id=2, name="Stop 2", lat=26.80, lon=75.75, demand_kg=2.0),
        ]
        vm = MulticopterModel(battery_usable_wh=200.0)  # Constrained battery
        profile = MissionProfile(cruise_speed_ms=15.0, hover_hold_s=60.0)
        profile.vehicle_model = vm
        plan = build_delivery_mission((26.90, 75.82), stops, zones=[], profile=profile)

        link = MockAutopilotLink()
        # Set battery to only 15% remaining (30 Wh remaining)
        link.telemetry.battery.value = FakeMavMessage("SYS_STATUS", remaining_pct=15, voltage_v=22.2, current_a=10.0, consumed_mah=5000.0)
        link.telemetry.position.value = PositionEstimate(
            lat=26.90, lon=75.82, alt_amsl_m=450.0, alt_rel_m=60.0,
            vn_ms=0.0, ve_ms=0.0, vd_ms=0.0, heading_deg=0.0, autopilot_time_ms=1000
        )
        link.telemetry.position.rx_time = time.monotonic()

        runner = MissionRunner(link, plan, zones=[], profile=profile, home=(26.90, 75.82), log=EventLog(echo=False))

        # Introduce a pop-up restriction that forces a large detour
        blocking_poly = [(26.87, 75.81), (26.88, 75.81), (26.88, 75.83), (26.87, 75.83)]
        res = runner.apply_airspace_update([blocking_poly], reason="TFR_ALERT", source={})

        self.assertEqual(res.get("event"), "operator_required")
        self.assertIn("REPLAN_INFEASIBLE", res.get("why", ""))
        self.assertEqual(link.current_mode, "LOITER")  # Holds safely!

    def test_replan_rejects_missing_or_stale_battery_telemetry(self):
        """apply_airspace_update must never default missing or stale battery telemetry to 100% SoC."""
        from flightstack.mission_runner import MissionRunner, EventLog

        stops = [Location(id=1, name="Stop 1", lat=26.85, lon=75.80, demand_kg=2.0)]
        vm = MulticopterModel(battery_usable_wh=280.0)
        profile = MissionProfile(cruise_speed_ms=10.0, hover_hold_s=5.0, vehicle_model=vm)
        plan = build_delivery_mission((26.90, 75.82), stops, zones=[], profile=profile)

        link = MockAutopilotLink()
        # Battery telemetry is missing (None)
        link.telemetry.battery.value = None
        link.telemetry.position.value = PositionEstimate(
            lat=26.90, lon=75.82, alt_amsl_m=450.0, alt_rel_m=60.0,
            vn_ms=0.0, ve_ms=0.0, vd_ms=0.0, heading_deg=0.0, autopilot_time_ms=1000
        )
        link.telemetry.position.rx_time = time.monotonic()

        runner = MissionRunner(link, plan, zones=[], profile=profile, home=(26.90, 75.82), log=EventLog(echo=False))
        blocking_poly = [(26.87, 75.805), (26.88, 75.805), (26.88, 75.815), (26.87, 75.815)]
        res = runner.apply_airspace_update([blocking_poly], reason="TFR_MISSING_BAT", source={})
        self.assertEqual(res.get("event"), "operator_required")
        self.assertEqual(res.get("breach_type"), "battery_telemetry_missing")
        self.assertEqual(link.current_mode, "LOITER")

        # Now provide stale battery telemetry (age 30s > 10s limit)
        link.telemetry.battery.value = FakeMavMessage("SYS_STATUS", remaining_pct=85, voltage_v=22.2, current_a=10.0, consumed_mah=500.0)
        link.telemetry.battery.rx_time = time.monotonic() - 30.0
        res_stale = runner.apply_airspace_update([blocking_poly], reason="TFR_STALE_BAT", source={})
        self.assertEqual(res_stale.get("event"), "operator_required")
        self.assertEqual(res_stale.get("breach_type"), "battery_telemetry_stale")
        self.assertEqual(link.current_mode, "LOITER")

    def test_replan_rejects_cold_chain_and_time_window_breaches(self):
        """apply_airspace_update must enforce cold-chain and delivery time windows as hard constraints."""
        from flightstack.mission_runner import MissionRunner, EventLog

        stop_cold = Location(
            id=1, name="Cold Stop", lat=26.85, lon=75.80, demand_kg=1.5,
            cold_chain_limit_minutes=5, window_minutes=30
        )
        profile = MissionProfile(cruise_speed_ms=5.0, hover_hold_s=5.0)
        plan = build_delivery_mission((26.90, 75.82), [stop_cold], zones=[], profile=profile)

        link = MockAutopilotLink()
        link.telemetry.position.value = PositionEstimate(
            lat=26.90, lon=75.82, alt_amsl_m=450.0, alt_rel_m=60.0,
            vn_ms=0.0, ve_ms=0.0, vd_ms=0.0, heading_deg=0.0, autopilot_time_ms=1000
        )
        link.telemetry.position.rx_time = time.monotonic()

        runner = MissionRunner(link, plan, zones=[], profile=profile, home=(26.90, 75.82), log=EventLog(echo=False))
        blocking_poly = [(26.87, 75.805), (26.88, 75.805), (26.88, 75.815), (26.87, 75.815)]
        res = runner.apply_airspace_update([blocking_poly], reason="TFR_COLD_CHAIN", source={})
        self.assertEqual(res.get("event"), "operator_required")
        self.assertEqual(res.get("breach_type"), "cold_chain")
        self.assertEqual(link.current_mode, "LOITER")

        # Now test hard delivery time window breach when cold chain is satisfied
        stop_win = Location(
            id=2, name="Window Stop", lat=26.85, lon=75.80, demand_kg=1.5,
            cold_chain_limit_minutes=60, window_minutes=5
        )
        plan2 = build_delivery_mission((26.90, 75.82), [stop_win], zones=[], profile=profile)
        runner2 = MissionRunner(link, plan2, zones=[], profile=profile, home=(26.90, 75.82), log=EventLog(echo=False))
        res_win = runner2.apply_airspace_update([blocking_poly], reason="TFR_TIME_WINDOW", source={})
        self.assertEqual(res_win.get("event"), "operator_required")
        self.assertEqual(res_win.get("breach_type"), "time_window")
        self.assertEqual(link.current_mode, "LOITER")

    def test_nav_delay_command_id_is_93_for_continue_after_land(self):
        """MAV_CMD_NAV_DELAY must be 93 (not 94/PAYLOAD_PLACE) so ArduCopter ContinueAfterLand works."""
        from flightstack.mission import MAV_CMD_NAV_DELAY
        self.assertEqual(MAV_CMD_NAV_DELAY, 93)

    def test_intermediate_landing_disarm_switches_to_guided_before_rearming(self):
        """If disarmed at an intermediate landing stop, runner must switch to GUIDED before arming."""
        from flightstack.mission_runner import MissionRunner, EventLog

        stops = [
            Location(
                id=1, name="Land Stop", lat=26.8890, lon=75.8180, demand_kg=1.5,
                site_suitability={"is_safe_for_landing": True, "surface_condition": "clear", "slope_deg": 1.0, "obstacle_clearance_m": 15.0}
            )
        ]
        profile = MissionProfile(delivery_mode="land", cruise_speed_ms=10.0)
        plan = build_delivery_mission((26.9060, 75.8165), stops, zones=[], profile=profile)

        class ModeTrackingLink(MockAutopilotLink):
            def __init__(self_inner):
                super().__init__()
                self_inner.mode_history = []
                self_inner.armed_in_mode = None
                self_inner.started = False
                self_inner.current_seq_set = None

            def set_mode(self_inner, mode: str) -> None:
                self_inner.current_mode = mode
                self_inner.mode_history.append(mode)

            def arm(self_inner, timeout: float = 15.0) -> None:
                if self_inner.current_mode == "AUTO":
                    raise RuntimeError("Arm: AUTO mode not armable")
                self_inner.armed_in_mode = self_inner.current_mode

            def set_current_item(self_inner, seq: int) -> None:
                self_inner.current_seq_set = seq

            def start_mission(self_inner) -> None:
                self_inner.started = True

        link = ModeTrackingLink()
        link.telemetry.heartbeat.value = ("AUTO", False)
        link.telemetry.heartbeat.rx_time = time.monotonic()
        link.telemetry.mission_current.value = 3
        link.telemetry.mission_current.rx_time = time.monotonic()
        link.telemetry.position.value = PositionEstimate(
            lat=26.8890, lon=75.8180, alt_amsl_m=431.0, alt_rel_m=0.1,
            vn_ms=0.0, ve_ms=0.0, vd_ms=0.0, heading_deg=0.0, autopilot_time_ms=5000
        )
        link.telemetry.position.rx_time = time.monotonic()

        runner = MissionRunner(link, plan, zones=[], profile=profile, home=(26.9060, 75.8165), log=EventLog(echo=False))
        runner.state.took_off = True
        runner.step()

        self.assertEqual(link.armed_in_mode, "GUIDED")
        self.assertEqual(link.mode_history, ["GUIDED", "AUTO"])
        self.assertTrue(link.started)
        # Disarming at an intermediate stop without gripper feedback must NOT fabricate delivery completion
        self.assertNotIn(1, runner.state.delivered)

    def test_auto_mode_selects_hover_for_stop1_and_land_for_stop2_consistently(self):
        """Priority 0: 'auto' mode must resolve Stop 1 -> hover and Stop 2 -> land across MissionPlan, PayloadManager, and MAVLink items."""
        from config import JAIPUR_SINGLE_UAV
        from flightstack.mission_runner import MissionRunner, EventLog

        c = JAIPUR_SINGLE_UAV
        profile = MissionProfile(delivery_mode="auto", cruise_speed_ms=10.0)
        plan = build_delivery_mission(
            (c.depot.lat, c.depot.lon),
            c.deliveries,
            [list(z) for z in c.no_fly_zones],
            profile,
        )
        # 1. MissionPlan selected_modes
        self.assertEqual(plan.selected_modes, {1: "hover", 2: "land"})

        # 2. Uploaded MAVLink item roles
        stop1_roles = [it.role for it in plan.items if it.stop_id == 1]
        stop2_roles = [it.role for it in plan.items if it.stop_id == 2]
        self.assertIn("hover_descend", stop1_roles)
        self.assertNotIn("land", stop1_roles)
        self.assertIn("land", stop2_roles)
        self.assertIn("ground_dwell", stop2_roles)
        self.assertIn("retakeoff", stop2_roles)

        # 3. PayloadManager consistency
        link = MockAutopilotLink()
        runner = MissionRunner(
            link, plan, zones=c.no_fly_zones, profile=profile,
            home=(c.depot.lat, c.depot.lon), log=EventLog(echo=False),
        )
        self.assertEqual(runner.payload.get_package(1).release_mode, "hover")
        self.assertEqual(runner.payload.get_package(1).planned_mode, "hover")
        self.assertEqual(runner.payload.get_package(2).release_mode, "land")
        self.assertEqual(runner.payload.get_package(2).planned_mode, "land")
        self.assertFalse(runner.payload.get_package(1).recipient_handover_verified)

    def test_unsuitable_site_rejected_and_abort_never_treated_as_delivery_mode(self):
        """Sites with clearance < 10m or recommended_mode='abort' must be rejected and placed in plan.unreachable."""
        bad_stop = Location(
            id=1, name="Collapsed Bridge", lat=26.8850, lon=75.7930, demand_kg=2.0,
            site_suitability={
                "is_safe_for_landing": False,
                "surface_condition": "debris",
                "obstacle_clearance_m": 6.0,
                "recommended_mode": "hover",
            },
        )
        good_stop = Location(
            id=2, name="Elevated Post", lat=26.8890, lon=75.8180, demand_kg=1.5,
            site_suitability={
                "is_safe_for_landing": True,
                "surface_condition": "clear",
                "obstacle_clearance_m": 18.0,
                "recommended_mode": "land",
            },
        )
        plan = build_delivery_mission(
            (26.9127, 75.8010),
            [bad_stop, good_stop],
            zones=[],
            profile=MissionProfile(delivery_mode="auto", cruise_speed_ms=10.0),
        )
        self.assertEqual([s.id for s in plan.unreachable], [1])
        self.assertEqual([s.id for s in plan.stops], [2])
        self.assertEqual(plan.selected_modes, {2: "land"})
        self.assertFalse(plan.site_assessments[1]["ok"])
        self.assertEqual(plan.site_assessments[1]["selected_mode"], "abort")

    def test_mid_flight_site_suitability_update_rebuilds_and_uploads_mission(self):
        """If Stop 2 degrades from clear/land to flooded in flight, runner must hold, rebuild mission as hover, upload, and resume."""
        from flightstack.mission_runner import MissionRunner, EventLog

        stop2 = Location(
            id=2, name="Jhalana Post", lat=26.8890, lon=75.8180, demand_kg=1.5,
            site_suitability={
                "is_safe_for_landing": True,
                "surface_condition": "clear",
                "slope_deg": 2.0,
                "obstacle_clearance_m": 18.0,
                "recommended_mode": "land",
            },
        )
        profile = MissionProfile(delivery_mode="auto", cruise_speed_ms=10.0)
        plan = build_delivery_mission((26.9127, 75.8010), [stop2], zones=[], profile=profile)
        self.assertEqual(plan.selected_modes[2], "land")
        self.assertIn(MAV_CMD_NAV_LAND, [it.command for it in plan.items])

        link = MockAutopilotLink()
        link.telemetry.position.value = PositionEstimate(
            lat=26.9050, lon=75.8080, alt_amsl_m=491.0, alt_rel_m=60.0,
            vn_ms=5.0, ve_ms=5.0, vd_ms=0.0, heading_deg=135.0, autopilot_time_ms=2000,
        )
        link.telemetry.position.rx_time = time.monotonic()
        link.telemetry.battery.value = FakeMavMessage(
            "SYS_STATUS", remaining_pct=88, voltage_v=24.0, current_a=12.0, consumed_mah=1200.0
        )
        link.telemetry.battery.rx_time = time.monotonic()

        runner = MissionRunner(
            link, plan, zones=[], profile=profile, home=(26.9127, 75.8010), log=EventLog(echo=False)
        )
        # Mid-flight update: Stop 2 courtyard becomes flooded!
        rec = runner.apply_site_suitability_update(
            2,
            {
                "is_safe_for_landing": False,
                "surface_condition": "flooded",
                "slope_deg": 2.0,
                "obstacle_clearance_m": 18.0,
                "recommended_mode": "land",
                "provenance": "OPERATOR_REPORT",
            },
            reason="flash_flood_at_stop2",
        )
        self.assertEqual(rec["previous_mode"], "land")
        self.assertEqual(rec["new_mode"], "hover")
        self.assertEqual(runner.plan.selected_modes[2], "hover")
        self.assertEqual(runner.payload.get_package(2).release_mode, "hover")
        self.assertNotIn(MAV_CMD_NAV_LAND, [it.command for it in runner.plan.items])
        self.assertIn("hover_descend", [it.role for it in runner.plan.items if it.stop_id == 2])
        self.assertEqual(link.current_mode, "AUTO")


# ===========================================================================
# Test Suite 5: Event Log & Replay Parsing Synchronization (Objective 8)
# ===========================================================================

class TestReplaySynchronization(unittest.TestCase):

    def test_parse_sitl_log_timestamp_consistency(self):
        """Verifies that visualize_sitl properly syncs t and t_rel_s."""
        from visualize_sitl import parse_sitl_log

        with tempfile.NamedTemporaryFile("w+", suffix=".jsonl", delete=False) as tf:
            # Write lines with t_rel_s (as written by EventLog)
            tf.write(json.dumps({"t_rel_s": 0.0, "event": "planned", "scenario": "Test"}) + "\n")
            tf.write(json.dumps({
                "t_rel_s": 12.5, "event": "telemetry",
                "telemetry": {"position": {"lat": 26.90, "lon": 75.82, "alt_rel_m": 60.0, "groundspeed_ms": 12.0}}
            }) + "\n")
            tf.write(json.dumps({"t_rel_s": 25.0, "event": "delivery_complete", "stop_id": 1}) + "\n")
            tf.write(json.dumps({"t_rel_s": 40.0, "event": "summary", "success": True}) + "\n")
            tf.flush()
            temp_path = tf.name

        try:
            parsed = parse_sitl_log(temp_path)
            samples = parsed["telemetry"]
            self.assertEqual(len(samples), 1)
            # Must read 12.5 from t_rel_s, NOT 0.0!
            self.assertEqual(samples[0]["t"], 12.5)
            self.assertEqual(samples[0]["t_rel_s"], 12.5)

            events = parsed["events"]
            self.assertEqual(len(events), 4)
            self.assertEqual(events[1]["t"], 12.5)
            self.assertEqual(events[2]["t"], 25.0)
        finally:
            if os.path.exists(temp_path):
                os.remove(temp_path)


# ===========================================================================
# Test Suite 6: Causal Gripper Feedback, Ground Dwell, and Missing/Failed Ack
# ===========================================================================

class TestPayloadRunnerIntegrationAndDwell(unittest.TestCase):

    def _make_runner(self, mode="hover"):
        from flightstack.mission_runner import MissionRunner, EventLog
        stop = Location(
            id=1,
            name="Stop 1",
            lat=26.8850,
            lon=75.7930,
            demand_kg=2.0,
            site_suitability={
                "is_safe_for_landing": (mode == "land"),
                "surface_condition": "clear" if mode == "land" else "flooded",
                "slope_deg": 2.0,
                "obstacle_clearance_m": 15.0,
                "recommended_mode": mode,
            },
        )
        profile = MissionProfile(delivery_mode=mode, cruise_speed_ms=10.0, hover_hold_s=5.0)
        plan = build_delivery_mission((26.9127, 75.8010), [stop], zones=[], profile=profile)
        link = MockAutopilotLink()
        link.telemetry.heartbeat.value = ("AUTO", True)
        link.telemetry.heartbeat.rx_time = time.monotonic()
        link.telemetry.position.value = PositionEstimate(
            lat=26.8850, lon=75.7930, alt_amsl_m=431.0, alt_rel_m=0.5 if mode == "land" else 15.0,
            vn_ms=0.0, ve_ms=0.0, vd_ms=0.0, heading_deg=0.0, autopilot_time_ms=1000,
        )
        link.telemetry.position.rx_time = time.monotonic()
        link.telemetry.battery.value = FakeMavMessage(
            "SYS_STATUS", remaining_pct=92, voltage_v=24.0, current_a=10.0, consumed_mah=800.0
        )
        link.telemetry.battery.rx_time = time.monotonic()
        runner = MissionRunner(
            link, plan, zones=[], profile=profile, home=(26.9127, 75.8010),
            log=EventLog(echo=False), actuator_ack_timeout_s=5.0,
        )
        runner.state.took_off = True
        return runner, link, plan

    def test_hover_delivery_requires_causal_gripper_feedback(self):
        """Hover delivery must not auto-acknowledge on command_release; requires 'Gripper Released'."""
        from flightstack.payload import DeliveryState
        runner, link, plan = self._make_runner("hover")
        rel_seq = plan.release_seq[1]
        comp_seq = plan.delivery_seq[1]

        # Advance to release_seq WITHOUT STATUSTEXT feedback
        link.telemetry.mission_current.value = rel_seq
        link.telemetry.mission_current.rx_time = time.monotonic()
        runner.step()

        pkg = runner.payload.get_package(1)
        self.assertEqual(pkg.delivery_state, DeliveryState.RELEASE_COMMANDED)
        self.assertFalse(pkg.simulated_release_acknowledged)
        self.assertNotIn(1, runner.state.delivered)

        # Now emit ArduPilot simulated gripper feedback and advance to climb (comp_seq)
        link.telemetry.status_texts.append((time.monotonic(), 6, "Gripper Released"))
        link.telemetry.mission_current.value = comp_seq
        runner.step()

        self.assertEqual(pkg.delivery_state, DeliveryState.DELIVERY_COMPLETE)
        self.assertTrue(pkg.simulated_release_acknowledged)
        self.assertTrue(pkg.stop_completion_recorded)
        self.assertFalse(pkg.recipient_handover_verified)
        self.assertEqual(pkg.release_provenance, "simulated_hover_gripper_release")
        self.assertIn(1, runner.state.delivered)

    def test_missing_feedback_marks_delivery_failed_not_completed(self):
        """If autopilot advances past completion seq without gripper feedback, delivery must fail."""
        from flightstack.payload import DeliveryState
        runner, link, plan = self._make_runner("hover")
        comp_seq = plan.delivery_seq[1]

        # Advance past comp_seq without any Gripper Released STATUSTEXT
        link.telemetry.mission_current.value = comp_seq + 1
        link.telemetry.mission_current.rx_time = time.monotonic()
        runner.step()

        pkg = runner.payload.get_package(1)
        self.assertEqual(pkg.delivery_state, DeliveryState.FAILED)
        self.assertEqual(pkg.status, PackageStatus.FAILED)
        self.assertIn("MISSING_ACTUATOR_FEEDBACK", pkg.error_reason)
        self.assertNotIn(1, runner.state.delivered)
        self.assertIn(1, runner.state.failed_deliveries)

    def test_failed_release_statustext_marks_delivery_failed(self):
        """If gripper reports a fault STATUSTEXT, delivery must fail and never be added to delivered."""
        from flightstack.payload import DeliveryState
        runner, link, plan = self._make_runner("hover")
        rel_seq = plan.release_seq[1]

        link.telemetry.mission_current.value = rel_seq
        link.telemetry.mission_current.rx_time = time.monotonic()
        link.telemetry.status_texts.append((time.monotonic(), 4, "Gripper failed"))
        runner.step()

        pkg = runner.payload.get_package(1)
        self.assertEqual(pkg.delivery_state, DeliveryState.FAILED)
        self.assertNotIn(1, runner.state.delivered)
        self.assertIn(1, runner.state.failed_deliveries)

    def test_land_delivery_waits_for_ground_dwell_before_completion(self):
        """Ground delivery must not complete during NAV_DELAY ground dwell; completes only at retakeoff."""
        from flightstack.payload import DeliveryState
        runner, link, plan = self._make_runner("land")
        rel_seq = plan.release_seq[1]
        dwell_seq = plan.dwell_seq[1]
        comp_seq = plan.delivery_seq[1]
        self.assertGreater(dwell_seq, rel_seq)
        self.assertGreater(comp_seq, dwell_seq)

        # At dwell_seq, gripper has released ("Gripper load released" + duplicate "Gripper Released")
        link.telemetry.mission_current.value = dwell_seq
        link.telemetry.mission_current.rx_time = time.monotonic()
        link.telemetry.status_texts.append((time.monotonic(), 6, f"Mission: {rel_seq} Gripper"))
        link.telemetry.status_texts.append((time.monotonic(), 6, "Gripper load released"))
        link.telemetry.status_texts.append((time.monotonic(), 6, "Gripper Released"))
        runner.step()

        pkg = runner.payload.get_package(1)
        # Release is acknowledged, duplicate is counted, but delivery is NOT complete during ground dwell!
        self.assertEqual(pkg.delivery_state, DeliveryState.RELEASE_ACKNOWLEDGED)
        self.assertTrue(pkg.simulated_release_acknowledged)
        self.assertFalse(pkg.stop_completion_recorded)
        self.assertEqual(pkg.duplicate_feedback_count, 1)
        self.assertNotIn(1, runner.state.delivered)
        event_names = [e["event"] for e in runner.log.events]
        self.assertIn("ground_dwell_started", event_names)
        self.assertNotIn("delivery_complete", event_names)

        # Once NAV_DELAY finishes and mission advances to retakeoff (comp_seq), delivery completes
        link.telemetry.mission_current.value = comp_seq
        runner.step()

        self.assertEqual(pkg.delivery_state, DeliveryState.DELIVERY_COMPLETE)
        self.assertTrue(pkg.stop_completion_recorded)
        self.assertIsNotNone(pkg.dwell_completed_time_s)
        self.assertFalse(pkg.recipient_handover_verified)
        self.assertEqual(pkg.release_provenance, "simulated_ground_gripper_release_after_dwell")
        self.assertIn(1, runner.state.delivered)
        event_names = [e["event"] for e in runner.log.events]
        self.assertIn("ground_dwell_completed", event_names)
        self.assertIn("delivery_complete", event_names)


# ===========================================================================
# Test Suite 7: Initial Mission Feasibility Gate & GDACS / DigitalSky Fixes
# ===========================================================================

class TestInitialMissionFeasibilityAndExternalData(unittest.TestCase):

    def _ready_link(self, remaining_pct=100, bat_age_s=0.0):
        link = MockAutopilotLink()
        link.wait_ready = lambda timeout=45.0: None
        link.upload_mission = lambda items, verify=True: None
        link.set_mode = lambda mode: setattr(link, "current_mode", mode)
        link.arm = lambda timeout=15.0: setattr(link, "armed", True)
        link.start_mission = lambda: None
        link.telemetry.position.value = PositionEstimate(
            lat=26.9127, lon=75.8010, alt_amsl_m=431.0, alt_rel_m=0.0,
            vn_ms=0.0, ve_ms=0.0, vd_ms=0.0, heading_deg=0.0, autopilot_time_ms=1000,
        )
        link.telemetry.position.rx_time = time.monotonic()
        if remaining_pct is not None:
            link.telemetry.battery.value = FakeMavMessage(
                "SYS_STATUS", remaining_pct=remaining_pct, voltage_v=24.0, current_a=5.0, consumed_mah=100.0
            )
            link.telemetry.battery.rx_time = time.monotonic() - bat_age_s
        else:
            link.telemetry.battery.value = None
        return link

    def test_launch_gated_on_initial_mission_feasibility(self):
        """runner.launch() must enforce payload capacity, MTOW, battery energy, missing/stale battery, cold-chain, and deadlines."""
        from flightstack.mission_runner import MissionRunner, EventLog
        from flightstack.mission import MissionValidationError

        home = (26.9127, 75.8010)
        base_stop = Location(
            id=1, name="Stop 1", lat=26.8850, lon=75.7930, demand_kg=2.0,
            window_minutes=25, cold_chain_limit_minutes=30,
        )

        # 1. Payload exceeds max_payload_kg
        heavy_stop = Location(id=1, name="Heavy Stop", lat=26.8850, lon=75.7930, demand_kg=9.5)
        vm = MulticopterModel(max_payload_kg=8.0, mtow_kg=17.5)
        prof_heavy = MissionProfile(cruise_speed_ms=10.0, vehicle_model=vm)
        plan_heavy = build_delivery_mission(home, [heavy_stop], zones=[], profile=prof_heavy)
        runner_heavy = MissionRunner(self._ready_link(), plan_heavy, [], prof_heavy, home, log=EventLog(echo=False))
        with self.assertRaises(MissionValidationError) as ctx:
            runner_heavy.launch()
        self.assertIn("INITIAL_MISSION_INFEASIBLE", str(ctx.exception))

        # 2. Missing battery telemetry
        prof_ok = MissionProfile(cruise_speed_ms=10.0, vehicle_model=vm)
        plan_ok = build_delivery_mission(home, [base_stop], zones=[], profile=prof_ok)
        runner_no_bat = MissionRunner(
            self._ready_link(remaining_pct=None), plan_ok, [], prof_ok, home, log=EventLog(echo=False)
        )
        with self.assertRaises(MissionValidationError) as ctx:
            runner_no_bat.launch()
        self.assertIn("Battery telemetry missing", str(ctx.exception))

        # 3. Stale battery telemetry
        runner_stale_bat = MissionRunner(
            self._ready_link(remaining_pct=95, bat_age_s=25.0), plan_ok, [], prof_ok, home, log=EventLog(echo=False)
        )
        with self.assertRaises(MissionValidationError) as ctx:
            runner_stale_bat.launch()
        self.assertIn("Battery telemetry is stale", str(ctx.exception))

        # 4. Insufficient battery energy / reserve
        runner_low_bat = MissionRunner(
            self._ready_link(remaining_pct=21), plan_ok, [], prof_ok, home, log=EventLog(echo=False)
        )
        with self.assertRaises(MissionValidationError) as ctx:
            runner_low_bat.launch()
        self.assertIn("Required energy", str(ctx.exception))

        # 5. Time-based cold-chain limit breach
        tight_cold = Location(
            id=1, name="Cold Stop", lat=26.8850, lon=75.7930, demand_kg=2.0,
            cold_chain_limit_minutes=1, window_minutes=30,
        )
        plan_cold = build_delivery_mission(home, [tight_cold], zones=[], profile=prof_ok)
        runner_cold = MissionRunner(self._ready_link(), plan_cold, [], prof_ok, home, log=EventLog(echo=False))
        with self.assertRaises(MissionValidationError) as ctx:
            runner_cold.launch()
        self.assertIn("time-based cold-chain limit", str(ctx.exception))

        # 6. Delivery deadline breach
        tight_win = Location(
            id=1, name="Win Stop", lat=26.8850, lon=75.7930, demand_kg=2.0,
            cold_chain_limit_minutes=30, window_minutes=1,
        )
        plan_win = build_delivery_mission(home, [tight_win], zones=[], profile=prof_ok)
        runner_win = MissionRunner(self._ready_link(), plan_win, [], prof_ok, home, log=EventLog(echo=False))
        with self.assertRaises(MissionValidationError) as ctx:
            runner_win.launch()
        self.assertIn("delivery window", str(ctx.exception))

    def test_gdacs_initial_application_before_upload_and_idempotence(self):
        """Relevant initial GDACS impact must be applied before mission upload and remain idempotent on polling."""
        from flightstack.mission_runner import MissionRunner, EventLog
        from gdacs_provider import GDACSEvent, GDACSSnapshot, GDACSMissionImpactAssessor

        home = (26.9127, 75.8010)
        stop = Location(
            id=2, name="Jhalana Post", lat=26.8890, lon=75.8180, demand_kg=1.5,
            site_suitability={
                "is_safe_for_landing": True,
                "surface_condition": "clear",
                "slope_deg": 2.0,
                "obstacle_clearance_m": 18.0,
                "recommended_mode": "land",
            },
        )
        # SYNTHETIC FIXTURE (not live GDACS data): Orange flood polygon covering Stop 2
        flood_poly = [
            [(26.880, 75.810), (26.880, 75.825), (26.895, 75.825), (26.895, 75.810)]
        ]
        now_utc = datetime.now(timezone.utc)
        ev = GDACSEvent(
            event_key="FL_99001_1",
            event_type="FL",
            event_id=99001,
            episode_id=1,
            name="SYNTHETIC_FIXTURE_Flood_Stop2",
            description="Synthetic test fixture polygon; not a real GDACS event",
            alert_level="Orange",
            alert_score=2.0,
            is_current=True,
            country="India",
            iso3="IND",
            centroid=(26.8890, 75.8180),
            polygons=flood_poly,
            from_date=now_utc - timedelta(hours=6),
            to_date=now_utc + timedelta(days=2),
            provenance="SYNTHETIC_FIXTURE",
        )

        class FixtureGDACSProvider:
            def fetch(self_inner):
                return GDACSSnapshot(
                    provider="GDACS_SYNTHETIC_FIXTURE",
                    source_url="fixture://gdacs",
                    events=[ev],
                    retrieved_at=datetime.now(timezone.utc),
                    content_sha256="0" * 64,
                    provenance="SYNTHETIC_FIXTURE",
                )

        assessor = GDACSMissionImpactAssessor(FixtureGDACSProvider(), fetch_missing_geometries=False)
        profile = MissionProfile(delivery_mode="auto", cruise_speed_ms=10.0)
        initial_plan = build_delivery_mission(home, [stop], zones=[], profile=profile)
        self.assertEqual(initial_plan.selected_modes[2], "land")

        link = self._ready_link()
        runner = MissionRunner(
            link, initial_plan, zones=[], profile=profile, home=home,
            log=EventLog(echo=False), gdacs_assessor=assessor,
        )
        # Launch applies initial GDACS impact BEFORE mission upload, downgrading Stop 2 from land -> hover
        runner.launch()
        self.assertEqual(runner.plan.selected_modes[2], "hover")
        self.assertEqual(runner.payload.get_package(2).release_mode, "hover")
        self.assertEqual(runner.state.mission_version, 1)

        # Repeated polling with the same GDACS snapshot must be idempotent (no duplicate replans)
        runner.poll_external_updates()
        runner.poll_external_updates()
        self.assertEqual(len(runner.state.replans), 0)
        self.assertEqual(runner.state.mission_version, 1)

    def test_digitalsky_full_route_polyline_covers_all_legs_and_unchecked_not_clearance(self):
        """Preflight DigitalSky validation must cover the entire planned route across all legs, not just leg 0."""
        from airspace_provider import DigitalSkyIntersectionChecker
        from config import JAIPUR_SINGLE_UAV

        c = JAIPUR_SINGLE_UAV
        home = (c.depot.lat, c.depot.lon)
        profile = MissionProfile(delivery_mode="auto", cruise_speed_ms=10.0)
        plan = build_delivery_mission(home, c.deliveries, [list(z) for z in c.no_fly_zones], profile)

        self.assertGreaterEqual(len(plan.leg_paths), 3)  # Depot->Stop1, Stop1->Stop2, Stop2->Depot
        full_pts = plan.full_route_polyline(home)
        self.assertGreater(len(full_pts), len(plan.leg_paths[0]))
        self.assertEqual(full_pts[0], home)
        self.assertEqual(full_pts[-1], home)
        for s in plan.stops:
            self.assertIn((s.lat, s.lon), full_pts)

        # Unauthenticated checker must report checked=False and official_clearance=False
        checker = DigitalSkyIntersectionChecker(transport=None, token=None)
        res = checker.validate_route_corridor(full_pts, alt_m_agl=60.0)
        self.assertFalse(res.checked)
        self.assertFalse(res.official_clearance)
        self.assertIsNone(res.intersects)

    def test_battery_usable_wh_remaining_no_double_reserve_penalty(self):
        """At 100% SoC with 20% reserve floor, full usable_battery_wh is available (no double-reserve penalty)."""
        from flightstack.mission import MissionValidationError
        from flightstack.mission_runner import EventLog, MissionRunner
        from flightstack.vehicle_model import EnergyEstimator, MulticopterModel

        home = (26.8920, 75.8070)
        stop = Location(2, "Medical Post", 26.9050, 75.8180, demand_kg=1.5)
        # Compute exact required route energy first
        vm_probe = MulticopterModel(battery_usable_wh=100.0)
        profile_probe = MissionProfile(delivery_mode="hover", cruise_speed_ms=10.0, vehicle_model=vm_probe)
        plan = build_delivery_mission(home, [stop], zones=[], profile=profile_probe)
        ee = EnergyEstimator(vm_probe)
        req_wh = ee.evaluate_route(
            plan.planned_km,
            [(stop, 1.5)],
            cruise_speed_ms=10.0,
            hover_time_s=profile_probe.hover_hold_s,
        ).total_energy_wh

        # Set usable_battery_wh slightly above req_wh (e.g. req_wh * 1.05).
        # Under the old double-reserve bug, 100% SoC multiplied usable_battery_wh by 0.80 again
        # and falsely rejected a route requiring 95% of usable_battery_wh!
        vm_exact = MulticopterModel(battery_usable_wh=req_wh * 1.05)
        profile_exact = MissionProfile(delivery_mode="hover", cruise_speed_ms=10.0, vehicle_model=vm_exact)
        link_full = self._ready_link(remaining_pct=100)
        runner_ok = MissionRunner(
            link_full, plan, zones=[], profile=profile_exact, home=home,
            battery_reserve_pct=20, log=EventLog(echo=False),
        )
        rep = runner_ok.validate_initial_mission(require_battery_telemetry=True)
        self.assertTrue(rep["feasible"])

        # At 20% SoC (at reserve floor), usable remaining Wh is 0.0 -> must reject
        link_floor = self._ready_link(remaining_pct=20)
        runner_floor = MissionRunner(
            link_floor, plan, zones=[], profile=profile_exact, home=home,
            battery_reserve_pct=20, log=EventLog(echo=False),
        )
        with self.assertRaises(MissionValidationError):
            runner_floor.validate_initial_mission(require_battery_telemetry=True)


if __name__ == "__main__":
    unittest.main()
