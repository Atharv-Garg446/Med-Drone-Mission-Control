"""
Unit tests for vehicle physics/energy modeling, precision landing transforms,
and dynamic obstacle tracking.

Runs in pure Python standard library without requiring SITL, Gazebo, or GPUs.
"""

import math
import unittest

from flightstack.vehicle_model import (
    VehicleProfile,
    BatterySpec,
    EnergyEstimator,
    RouteEnergyReport,
    GRAVITY,
)
from flightstack.precision_landing import (
    CameraIntrinsics,
    TargetDetection,
    optical_to_body_ned,
    angles_from_camera_position,
    build_landing_target_mavlink_payload,
    PrecisionLandingDetector,
)
from flightstack.obstacle_tracker import (
    Vector3D,
    TrackedObstacle,
    ObstacleTracker,
    ConflictAlert,
    obstacle_to_exclusion_polygon,
)


class TestVehicleModel(unittest.TestCase):
    def setUp(self):
        self.vehicle = VehicleProfile()
        self.battery = BatterySpec()
        self.estimator = EnergyEstimator(self.vehicle, self.battery)

    def test_mass_accounting_and_mtow_enforcement(self):
        # Default: empty=5.5kg, battery=3.2kg, mtow=17.5kg, max_payload=8.0kg
        self.assertEqual(self.vehicle.total_mass(0.0), 8.7)
        self.assertEqual(self.vehicle.total_mass(5.0), 13.7)
        self.assertEqual(self.vehicle.total_mass(8.0), 16.7)

        # Exceeding payload limit raises
        with self.assertRaises(ValueError):
            self.vehicle.total_mass(9.0)

        # Negative payload raises
        with self.assertRaises(ValueError):
            self.vehicle.total_mass(-1.0)

    def test_invalid_airframe_raises(self):
        with self.assertRaises(ValueError):
            # Base mass > MTOW
            VehicleProfile(empty_mass_kg=12.0, battery_mass_kg=8.0, mtow_kg=15.0)

    def test_hover_power_mass_scaling(self):
        p_empty = self.vehicle.hover_power_watts(0.0)
        p_full = self.vehicle.hover_power_watts(8.0)
        # Power in hover should scale roughly as mass^1.5: (16.7 / 8.7)^1.5 ~= 2.65
        ratio = p_full / p_empty
        self.assertGreater(ratio, 2.2)
        self.assertLess(ratio, 3.0)

    def test_cruise_power_greater_with_payload(self):
        p_cruise_empty = self.vehicle.cruise_power_watts(0.0, airspeed_ms=11.11)
        p_cruise_full = self.vehicle.cruise_power_watts(8.0, airspeed_ms=11.11)
        self.assertGreater(p_cruise_full, p_cruise_empty)

    def test_route_energy_feasibility(self):
        # Route: 0 -> 1 -> 2 -> 0
        route = [0, 1, 2, 0]
        distances = [4.0, 3.0, 5.0]  # 12 km total
        demands = [0.0, 3.0, 2.0]    # 5 kg total initial cargo

        report = self.estimator.evaluate_route(route, distances, demands, cruise_speed_ms=11.11)
        self.assertTrue(report.is_feasible)
        self.assertTrue(report.mtow_ok)
        self.assertGreater(report.reserve_margin_pct, self.battery.reserve_fraction * 100)
        self.assertEqual(report.initial_cargo_kg, 5.0)
        self.assertEqual(len(report.legs), 3)

        # Leg 0 carries 5kg, Leg 1 carries 2kg, Leg 2 carries 0kg (empty return)
        self.assertEqual(report.legs[0].payload_carried_kg, 5.0)
        self.assertEqual(report.legs[1].payload_carried_kg, 2.0)
        self.assertEqual(report.legs[2].payload_carried_kg, 0.0)

        # Leg 0 should consume more power than Leg 2 over equivalent airspeed
        self.assertGreater(report.legs[0].cruise_power_w, report.legs[2].cruise_power_w)

    def test_excessive_distance_detected_as_infeasible(self):
        # 80 km route with heavy payload exceeds 16Ah battery usable energy
        route = [0, 1, 0]
        distances = [40.0, 40.0]
        demands = [0.0, 7.5]

        report = self.estimator.evaluate_route(route, distances, demands)
        self.assertFalse(report.is_feasible)
        self.assertIn("exceeds usable battery budget", report.failure_reason)


class TestPrecisionLanding(unittest.TestCase):
    def test_optical_to_body_ned_transformation(self):
        # Target located 1m right, 2m down, 5m below in camera optical frame:
        # Camera: xc=1.0 (right), yc=2.0 (down in image), zc=5.0 (depth along optical axis)
        # Body-NED: +X forward = -yc = -2.0m, +Y right = +xc = 1.0m, +Z down = +zc = 5.0m
        pos_cam = (1.0, 2.0, 5.0)
        pos_body = optical_to_body_ned(pos_cam)
        self.assertEqual(pos_body, (-2.0, 1.0, 5.0))

    def test_angles_calculation(self):
        # Target centered in frame at 10m depth:
        ang_x, ang_y, dist = angles_from_camera_position(0.0, 0.0, 10.0)
        self.assertAlmostEqual(ang_x, 0.0)
        self.assertAlmostEqual(ang_y, 0.0)
        self.assertAlmostEqual(dist, 10.0)

        # Target at 45 deg right (xc = zc):
        ang_x45, _, dist45 = angles_from_camera_position(10.0, 0.0, 10.0)
        self.assertAlmostEqual(ang_x45, math.pi / 4.0, places=4)
        self.assertAlmostEqual(dist45, math.sqrt(200.0), places=4)

    def test_landing_target_mavlink_payload_structure(self):
        detection = TargetDetection(
            detected=True,
            marker_id=42,
            pos_camera_m=(0.5, 0.2, 8.0),
            pos_body_m=(-0.2, 0.5, 8.0),
            angle_x_rad=0.0624,
            angle_y_rad=0.025,
            distance_m=8.018,
            reason="Target locked",
        )
        payload = build_landing_target_mavlink_payload(detection, time_usec=123456)
        self.assertIsNotNone(payload)
        self.assertEqual(payload["message_type"], "LANDING_TARGET")
        self.assertEqual(payload["x"], -0.2)
        self.assertEqual(payload["y"], 0.5)
        self.assertEqual(payload["z"], 8.0)
        self.assertEqual(payload["position_valid"], 1)

    def test_undetected_target_returns_none(self):
        undetected = TargetDetection(detected=False, reason="Obscured")
        self.assertIsNone(build_landing_target_mavlink_payload(undetected))

    def test_detector_fallback_without_cv2(self):
        detector = PrecisionLandingDetector()
        res = detector.detect_marker_in_image(None)
        self.assertFalse(res.detected)
        self.assertIn("reason", res.__dict__)


class TestObstacleTracker(unittest.TestCase):
    def setUp(self):
        self.tracker = ObstacleTracker(safety_margin_m=30.0, time_horizon_s=10.0)

    def test_vector3d_math(self):
        v1 = Vector3D(3.0, 4.0, 0.0)
        self.assertAlmostEqual(v1.norm(), 5.0)
        v2 = Vector3D(1.0, 1.0, 0.0)
        diff = v1.sub(v2)
        self.assertEqual((diff.x, diff.y, diff.z), (2.0, 3.0, 0.0))
        self.assertAlmostEqual(v1.dot(v2), 7.0)

    def test_head_on_collision_course_conflict(self):
        # Drone at (0, 0, -30), flying North at 10 m/s
        drone_pos = (0.0, 0.0, -30.0)
        drone_vel = (10.0, 0.0, 0.0)

        # Obstacle at (100, 0, -30), flying South at 10 m/s (closing speed = 20 m/s)
        # Collision at t = 100 / 20 = 5.0 seconds
        self.tracker.update_track("OBS_1", pos_ned_m=(100.0, 0.0, -30.0), vel_ned_ms=(-10.0, 0.0, 0.0))

        alerts = self.tracker.evaluate_conflict(drone_pos, drone_vel)
        self.assertEqual(len(alerts), 1)
        alert = alerts[0]
        self.assertTrue(alert.has_conflict)
        self.assertAlmostEqual(alert.t_cpa_s, 5.0, places=1)
        self.assertAlmostEqual(alert.d_cpa_m, 0.0, places=1)
        self.assertIn(alert.action_required, ["HOLD", "REROUTE"])

    def test_diverging_trajectory_no_conflict(self):
        # Drone flying North, obstacle flying away further North
        drone_pos = (0.0, 0.0, -30.0)
        drone_vel = (5.0, 0.0, 0.0)

        self.tracker.update_track("OBS_AWAY", pos_ned_m=(50.0, 0.0, -30.0), vel_ned_ms=(15.0, 0.0, 0.0))
        alerts = self.tracker.evaluate_conflict(drone_pos, drone_vel)
        self.assertFalse(alerts[0].has_conflict)
        self.assertEqual(alerts[0].action_required, "NONE")

    def test_perpendicular_miss(self):
        # Drone at (0,0), flying North (vy=0). Obstacle crosses east-west at x=50m, y=100m, vy=-20m/s
        # When drone reaches x=50m (t=5s), obstacle is at y = 100 - 20*5 = 0m -> Conflict!
        drone_pos = (0.0, 0.0, -30.0)
        drone_vel = (10.0, 0.0, 0.0)

        self.tracker.update_track("CROSS_HAZARD", pos_ned_m=(50.0, 100.0, -30.0), vel_ned_ms=(0.0, -20.0, 0.0))
        alerts = self.tracker.evaluate_conflict(drone_pos, drone_vel)
        self.assertTrue(alerts[0].has_conflict)
        self.assertAlmostEqual(alerts[0].t_cpa_s, 4.0, delta=1.5)

    def test_stale_track_purging(self):
        self.tracker.update_track("OBS_OLD", pos_ned_m=(10.0, 10.0, 0.0), timestamp_s=100.0)
        self.assertEqual(len(self.tracker.tracks), 1)
        purged = self.tracker.purge_stale_tracks(now=105.0)
        self.assertEqual(purged, 1)
        self.assertEqual(len(self.tracker.tracks), 0)

    def test_persistent_track_retained(self):
        self.tracker.update_track("CRANE_STATIC", pos_ned_m=(10.0, 10.0, 0.0), timestamp_s=100.0, is_persistent=True)
        self.assertEqual(len(self.tracker.tracks), 1)
        purged = self.tracker.purge_stale_tracks(now=200.0)
        self.assertEqual(purged, 0)
        self.assertEqual(len(self.tracker.tracks), 1)


    def test_exclusion_polygon_generation(self):
        poly = obstacle_to_exclusion_polygon(26.9, 75.8, radius_m=50.0, num_vertices=6)
        self.assertEqual(len(poly), 6)
        for lat, lon in poly:
            self.assertTrue(26.899 < lat < 26.901)
            self.assertTrue(75.799 < lon < 75.801)

    def test_geo_ned_roundtrip(self):
        from flightstack.obstacle_tracker import geo_to_ned, ned_to_geo
        origin = (26.900, 75.800)
        target = (26.905, 75.808, 30.0)
        n, e, d = geo_to_ned(target[0], target[1], target[2], origin[0], origin[1])
        lat2, lon2, alt2 = ned_to_geo(n, e, d, origin[0], origin[1])
        self.assertAlmostEqual(lat2, target[0], places=5)
        self.assertAlmostEqual(lon2, target[1], places=5)
        self.assertAlmostEqual(alt2, target[2], places=2)

    def test_update_track_geo(self):
        origin = (26.900, 75.800)
        tr = self.tracker.update_track_geo(
            "RADAR_HAZARD", lat=26.902, lon=75.802, alt_rel_m=40.0,
            origin_lat=origin[0], origin_lon=origin[1], radius_m=12.0
        )
        self.assertEqual(tr.obstacle_id, "RADAR_HAZARD")
        self.assertEqual(tr.radius_m, 12.0)
        self.assertAlmostEqual(tr.position_m.z, -40.0, places=1)


class TestVrpEnergyIntegration(unittest.TestCase):
    def test_is_route_feasible_with_energy_model(self):
        from config import DroneConfig, Location
        from vrp_scratch import is_route_feasible, _diagnose_unreachable_customer

        # Depot + 2 stops
        depot = Location(id=0, name="Depot", lat=26.90, lon=75.80)
        s1 = Location(id=1, name="Suburban Clinic", lat=26.95, lon=75.80, demand_kg=2.5)
        s2 = Location(id=2, name="Rural Outpost", lat=27.00, lon=75.80, demand_kg=1.5)
        locations = [depot, s1, s2]

        # 10 km round trip: 0 -> 1 (3km), 1 -> 2 (3km), 2 -> 0 (4km)
        matrix = [
            [0.0, 3.0, 4.0],
            [3.0, 0.0, 3.0],
            [4.0, 3.0, 0.0],
        ]

        # Standard drone: 8kg capacity, 50km range.
        drone = DroneConfig(capacity_kg=8.0, max_range_km=50.0, reserve_fraction=0.10)
        route = [0, 1, 2, 0]

        # Without energy model: distance is 10km <= 45km, capacity 4.0kg <= 8.0kg -> Feasible
        self.assertTrue(is_route_feasible(route, locations, matrix, drone))

        # Enable energy model on drone
        drone.enable_energy_model()
        self.assertIsNotNone(drone.energy_estimator)

        # 10km route with 4.0kg cargo consumes ~218 Wh <= 319.7 Wh usable -> Feasible
        self.assertTrue(is_route_feasible(route, locations, matrix, drone))

        # Now test a heavy route that is feasible by flat distance (20km <= 45km)
        # but fails the physics/energy model because 7.5kg cargo + hover dwells require 381 Wh > 320 Wh usable:
        heavy_s1 = Location(id=1, name="Suburban Clinic", lat=26.95, lon=75.80, demand_kg=6.0)
        heavy_locations = [depot, heavy_s1, s2]
        heavy_matrix = [
            [0.0, 5.0, 10.0],
            [5.0, 0.0, 5.0],
            [10.0, 5.0, 0.0],
        ]
        # Without energy model: 20km <= 45km, demand 7.5kg <= 8.0kg -> Passes
        drone_flat = DroneConfig(capacity_kg=8.0, max_range_km=50.0, reserve_fraction=0.10)
        self.assertTrue(is_route_feasible(route, heavy_locations, heavy_matrix, drone_flat))

        # With energy model enabled: passes distance, but rejected by energy exhaustion!
        drone_physics = DroneConfig(capacity_kg=8.0, max_range_km=50.0, reserve_fraction=0.10)
        drone_physics.enable_energy_model()
        self.assertFalse(is_route_feasible(route, heavy_locations, heavy_matrix, drone_physics))

        # And customer diagnosis explains the energy breach:
        diag = _diagnose_unreachable_customer(1, heavy_locations, heavy_matrix, drone_physics)
        self.assertIn("energy", diag.lower())



class TestPrecisionLandingSynthetic(unittest.TestCase):
    def test_synthetic_pad_detection(self):
        from flightstack.precision_landing import (
            PrecisionLandingDetector,
            generate_synthetic_landing_pad,
            build_landing_target_mavlink_payload,
            HAVE_CV2,
        )
        detector = PrecisionLandingDetector(target_marker_id=0)
        if HAVE_CV2:
            img = generate_synthetic_landing_pad(marker_id=0)
            self.assertIsNotNone(img)
            det = detector.detect_marker_in_image(img)
            self.assertTrue(det.detected)
            self.assertEqual(det.marker_id, 0)
            self.assertIsNotNone(det.pos_body_m)

            payload = build_landing_target_mavlink_payload(det)
            self.assertIsNotNone(payload)
            self.assertEqual(payload["message_type"], "LANDING_TARGET")
            self.assertEqual(payload["position_valid"], 1)
        else:
            det = detector.detect_marker_in_image(None)
            self.assertFalse(det.detected)
            self.assertIn("not available", det.reason.lower())


if __name__ == "__main__":
    unittest.main()

