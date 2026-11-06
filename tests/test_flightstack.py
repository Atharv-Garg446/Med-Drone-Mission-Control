"""
Fast tests for the flight-stack integration layer and the airspace adapter.
No SITL, network or pymavlink required: MAVLink messages and the autopilot
side of the mission protocol are replaced by small test doubles.
"""

import json
import os
import sys
import tempfile
import threading
import queue
import unittest
from datetime import datetime, timedelta, timezone

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import JAIPUR_DISASTER, Location
from geo_utils import segment_intersects_polygon
from flightstack import telemetry as T
from flightstack.mission import (MissionProfile, build_delivery_mission, MissionValidationError,
                                 validate_polyline, mission_to_qgc_wpl, MAV_CMD_DO_GRIPPER,
                                 MAV_CMD_NAV_LAND, MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_RETURN_TO_LAUNCH)
from flightstack.mavlink_link import AutopilotLink, LinkError
import airspace_provider as AP


class Msg:
    def __init__(self, mtype, src=1, **fields):
        self._t = mtype
        self._src = src
        self.__dict__.update(fields)

    def get_type(self):
        return self._t

    def get_srcSystem(self):
        return self._src


def gpi(lat=26.9, lon=75.8, alt=431000, rel=60000, t=1000):
    return Msg("GLOBAL_POSITION_INT", lat=int(lat * 1e7), lon=int(lon * 1e7), alt=alt,
               relative_alt=rel, vx=300, vy=400, vz=0, hdg=9000, time_boot_ms=t)


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class TestTelemetry(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.tel = T.TelemetryState(clock=self.clock)

    def test_position_decoding_and_groundspeed(self):
        self.tel.update(gpi())
        p = self.tel.fresh_position()
        self.assertAlmostEqual(p.lat, 26.9, places=6)
        self.assertAlmostEqual(p.alt_rel_m, 60.0)
        self.assertAlmostEqual(p.groundspeed_ms, 5.0)
        self.assertEqual(p.heading_deg, 90.0)

    def test_zero_position_rejected(self):
        self.tel.update(gpi(lat=0, lon=0))
        self.assertIsNone(self.tel.position.value)
        self.assertEqual(self.tel.rejected["invalid_position"], 1)
        with self.assertRaises(T.StaleTelemetryError):
            self.tel.fresh_position()

    def test_stale_position_raises(self):
        self.tel.update(gpi())
        self.clock.t += 2.5
        with self.assertRaises(T.StaleTelemetryError):
            self.tel.fresh_position(max_age_s=2.0)

    def test_no_3d_fix_rejects_position(self):
        self.tel.update(gpi())
        self.tel.update(Msg("GPS_RAW_INT", fix_type=2, satellites_visible=5, eph=300))
        with self.assertRaises(T.StaleTelemetryError):
            self.tel.fresh_position()

    def test_unhealthy_ekf_rejects_position(self):
        self.tel.update(gpi())
        self.tel.update(Msg("EKF_STATUS_REPORT", flags=T.EKF_ATTITUDE))
        with self.assertRaises(T.StaleTelemetryError):
            self.tel.fresh_position()
        self.tel.update(Msg("EKF_STATUS_REPORT",
                            flags=T.EKF_ATTITUDE | T.EKF_VELOCITY_HORIZ | T.EKF_POS_HORIZ_ABS))
        self.tel.fresh_position()

    def test_heartbeat_mode_armed_and_gcs_ignored(self):
        self.tel.update(Msg("HEARTBEAT", type=2, custom_mode=3, base_mode=128 | 1))
        self.assertEqual(self.tel.mode, "AUTO")
        self.assertTrue(self.tel.armed)
        self.tel.update(Msg("HEARTBEAT", type=6, custom_mode=0, base_mode=0))
        self.assertEqual(self.tel.mode, "AUTO")

    def test_battery_unknown_values_are_none(self):
        self.tel.update(Msg("SYS_STATUS", voltage_battery=65535, current_battery=-1,
                            battery_remaining=-1))
        b = self.tel.battery.value
        self.assertIsNone(b.voltage_v)
        self.assertIsNone(b.current_a)
        self.assertIsNone(b.remaining_pct)

    def test_snapshot_is_json_serialisable(self):
        self.tel.update(gpi())
        json.dumps(self.tel.snapshot())


class TestMissionBuilder(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        c = JAIPUR_DISASTER
        cls.home = (c.depot.lat, c.depot.lon)
        cls.zones = [list(z) for z in c.no_fly_zones]
        cls.stops = [c.deliveries[5], c.deliveries[3]]      # ids 6, 4

    def test_hover_mission_structure(self):
        plan = build_delivery_mission(self.home, self.stops, self.zones, MissionProfile())
        roles = [i.role for i in plan.items]
        self.assertEqual(roles[0], "home")
        self.assertEqual(roles[1], "takeoff")
        self.assertEqual(plan.items[-1].command, MAV_CMD_NAV_RETURN_TO_LAUNCH)
        self.assertEqual(sum(1 for i in plan.items if i.command == MAV_CMD_DO_GRIPPER), 2)
        self.assertEqual([i.seq for i in plan.items], list(range(len(plan.items))))
        for sid, seq in plan.delivery_seq.items():
            self.assertEqual(plan.items[seq].role, "climb")
            self.assertEqual(plan.items[seq].stop_id, sid)

    def test_land_mode_uses_land_then_takeoff(self):
        from flightstack.mission import MAV_CMD_NAV_DELAY
        plan = build_delivery_mission(self.home, self.stops, self.zones,
                                      MissionProfile(delivery_mode="land"))
        cmds = [i.command for i in plan.items]
        self.assertEqual(cmds.count(MAV_CMD_NAV_LAND), 2)
        self.assertEqual(cmds.count(MAV_CMD_DO_GRIPPER), 2)
        self.assertEqual(cmds.count(MAV_CMD_NAV_DELAY), 2)
        for sid, rel_seq in plan.release_seq.items():
            self.assertEqual(plan.items[rel_seq].command, MAV_CMD_DO_GRIPPER)
            dwell_seq = plan.dwell_seq[sid]
            comp_seq = plan.delivery_seq[sid]
            self.assertEqual(plan.items[dwell_seq].command, MAV_CMD_NAV_DELAY)
            self.assertEqual(plan.items[comp_seq].command, MAV_CMD_NAV_TAKEOFF)
            self.assertGreater(dwell_seq, rel_seq)
            self.assertGreater(comp_seq, dwell_seq)

    def test_per_stop_mode_override(self):
        plan = build_delivery_mission(self.home, self.stops, self.zones, MissionProfile(),
                                      stop_modes={4: "land"})
        self.assertEqual([i.command for i in plan.items].count(MAV_CMD_NAV_LAND), 1)

    def test_all_nav_segments_clear_zones_with_margin(self):
        plan = build_delivery_mission(self.home, self.stops, self.zones, MissionProfile())
        for path in plan.leg_paths:
            validate_polyline(path, self.zones, 50.0)

    def test_detour_inserted_when_leg_crosses_zone(self):
        zone = [(26.858, 75.811), (26.858, 75.818), (26.864, 75.818), (26.864, 75.811)]
        plan = build_delivery_mission(self.home, self.stops, self.zones + [zone], MissionProfile())
        self.assertTrue(any(i.role == "detour" for i in plan.items))
        for path in plan.leg_paths:
            for a, b in zip(path, path[1:]):
                self.assertFalse(segment_intersects_polygon(a, b, zone, margin_m=50.0))

    def test_unreachable_stop_skipped_not_straight_lined(self):
        s = self.stops[0]
        box = [(s.lat - 0.002, s.lon - 0.002), (s.lat - 0.002, s.lon + 0.002),
               (s.lat + 0.002, s.lon + 0.002), (s.lat + 0.002, s.lon - 0.002)]
        plan = build_delivery_mission(self.home, self.stops, self.zones + [box], MissionProfile())
        self.assertEqual([x.id for x in plan.unreachable], [s.id])
        self.assertNotIn(s.id, plan.delivery_seq)

    def test_validate_polyline_detects_violation(self):
        zone = [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0)]
        with self.assertRaises(MissionValidationError):
            validate_polyline([(-0.5, 0.5), (1.5, 0.5)], [zone], 0.0)

    def test_profile_validation(self):
        with self.assertRaises(ValueError):
            MissionProfile(delivery_mode="parachute")
        with self.assertRaises(ValueError):
            MissionProfile(cruise_alt_m=150)
        with self.assertRaises(ValueError):
            MissionProfile(release_alt_m=80, cruise_alt_m=60)

    def test_in_flight_update_has_no_takeoff(self):
        plan = build_delivery_mission(self.home, self.stops, self.zones, MissionProfile(),
                                      include_takeoff=False)
        self.assertNotIn("takeoff", [i.role for i in plan.items])

    def test_qgc_export_roundtrip_lines(self):
        plan = build_delivery_mission(self.home, self.stops, self.zones, MissionProfile())
        lines = mission_to_qgc_wpl(plan).strip().split("\n")
        self.assertEqual(lines[0], "QGC WPL 110")
        self.assertEqual(len(lines) - 1, len(plan.items))
        self.assertEqual(len(lines[1].split("\t")), 12)


class FakeMav:
    """Records outgoing calls and plays the autopilot side of the protocols."""

    def __init__(self, inbox, behaviour="ok"):
        self.inbox = inbox
        self.behaviour = behaviour
        self.items = {}
        self.count = None
        self.commands = []

    def mission_count_send(self, ts, tc, n, mtype):
        self.count = n
        if self.behaviour == "silent":
            return
        self.inbox.put(Msg("MISSION_REQUEST_INT", seq=0))

    def mission_item_int_send(self, ts, tc, seq, frame, cmd, cur, ac, p1, p2, p3, p4, x, y, z, mt):
        self.items[seq] = (cmd, x, y, z)
        if self.behaviour == "reject":
            self.inbox.put(Msg("MISSION_ACK", type=4))
        elif seq + 1 < self.count:
            self.inbox.put(Msg("MISSION_REQUEST_INT", seq=seq + 1))
        else:
            self.inbox.put(Msg("MISSION_ACK", type=0))

    def mission_request_list_send(self, ts, tc, mtype):
        self.inbox.put(Msg("MISSION_COUNT", count=len(self.items)))

    def mission_request_int_send(self, ts, tc, seq, mtype):
        cmd, x, y, z = self.items[seq]
        self.inbox.put(Msg("MISSION_ITEM_INT", seq=seq, command=cmd, x=x, y=y, z=z))

    def mission_ack_send(self, ts, tc, type_val, mtype):
        pass

    def mission_set_current_send(self, ts, tc, seq):
        self.inbox.put(Msg("MISSION_CURRENT", seq=seq))

    def command_long_send(self, ts, tc, cmd, conf, *p):
        self.commands.append((cmd, p))
        self.inbox.put(Msg("COMMAND_ACK", command=cmd, result=0))
        if cmd == 176 and len(p) >= 2:
            self.inbox.put(Msg("HEARTBEAT", custom_mode=int(p[1]), base_mode=128))

    def obstacle_distance_send(self, time_usec, sensor_type, distances, increment, min_dist, max_dist, *args):
        if not hasattr(self, "obstacle_distances"):
            self.obstacle_distances = []
        self.obstacle_distances.append((time_usec, distances, min_dist, max_dist))


class FakeConn:
    target_system = 1
    target_component = 1

    def __init__(self, behaviour="ok"):
        self.inbox = queue.Queue()
        self.mav = FakeMav(self.inbox, behaviour)

    def recv_match(self, blocking=True, timeout=0.2):
        try:
            return self.inbox.get(timeout=timeout)
        except queue.Empty:
            return None

    def close(self):
        pass


class TestMavlinkLink(unittest.TestCase):
    def _plan(self):
        c = JAIPUR_DISASTER
        return build_delivery_mission((c.depot.lat, c.depot.lon), [c.deliveries[5]],
                                      [list(z) for z in c.no_fly_zones], MissionProfile())

    def test_mission_upload_protocol(self):
        conn = FakeConn()
        link = AutopilotLink(conn)
        link.start()
        try:
            plan = self._plan()
            link.upload_mission(plan.items, item_timeout=1.0)
            self.assertEqual(sorted(conn.mav.items), list(range(len(plan.items))))
            cmd, x, y, z = conn.mav.items[2]
            self.assertEqual(x, int(round(plan.items[2].lat * 1e7)))
        finally:
            link.close()

    def test_mission_rejection_surfaces(self):
        conn = FakeConn("reject")
        link = AutopilotLink(conn)
        link.start()
        try:
            with self.assertRaises(LinkError):
                link.upload_mission(self._plan().items, item_timeout=1.0)
        finally:
            link.close()

    def test_silent_autopilot_times_out(self):
        conn = FakeConn("silent")
        link = AutopilotLink(conn)
        link.start()
        try:
            with self.assertRaises(LinkError):
                link.upload_mission(self._plan().items, item_timeout=0.2, retries=1)
        finally:
            link.close()

    def test_command_ack_and_foreign_system_filtered(self):
        conn = FakeConn()
        link = AutopilotLink(conn)
        link.start()
        try:
            self.assertEqual(link.command_long(400, 1), 0)
            conn.inbox.put(Msg("HEARTBEAT", src=42, type=2, custom_mode=3, base_mode=128))
            conn.inbox.put(Msg("HEARTBEAT", src=1, type=2, custom_mode=5, base_mode=0))
            link.wait_for(lambda t: t.mode is not None, 2.0)
            self.assertEqual(link.telemetry.mode, "LOITER")
        finally:
            link.close()


FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures", "airspace_sample.geojson")


class TestAirspaceProvider(unittest.TestCase):
    def test_geojson_ingest_categories_and_provenance(self):
        snap = AP.GeoJSONFileProvider(FIXTURE, "fixture").fetch()
        cats = sorted(z.category for z in snap.zones)
        self.assertEqual(cats, ["green", "red", "temporary_restriction", "yellow"])
        self.assertEqual(len(snap.content_sha256), 64)
        self.assertEqual(snap.version, "fixture-1")
        self.assertFalse(snap.official)
        lat, lon = snap.zones[0].polygon[0]
        self.assertTrue(26 < lat < 27 and 75 < lon < 76)    # lon/lat swapped correctly
        self.assertNotEqual(snap.zones[0].polygon[0], snap.zones[0].polygon[-1])

    def test_planning_polygons_respect_category_and_validity(self):
        snap = AP.GeoJSONFileProvider(FIXTURE, "fixture").fetch()
        inside = datetime(2026, 10, 26, 6, 0, tzinfo=timezone.utc)
        after = datetime(2026, 10, 27, 6, 0, tzinfo=timezone.utc)
        self.assertEqual(len(snap.planning_polygons(when=inside)), 3)   # red, yellow, TR
        self.assertEqual(len(snap.planning_polygons(when=after)), 2)    # TR expired
        self.assertEqual(len(snap.planning_polygons(blocking=("red",), when=inside)), 1)

    def test_invalid_geometry_rejected(self):
        bad = {"type": "FeatureCollection", "features": [{"type": "Feature",
               "properties": {"category": "red"},
               "geometry": {"type": "Polygon", "coordinates": [[[75.8, 126.9], [75.9, 26.9], [75.9, 27.0], [75.8, 126.9]]]}}]}
        with self.assertRaises(ValueError):
            AP.zones_from_geojson(bad)

    def test_unknown_category_rejected(self):
        doc = {"type": "FeatureCollection", "features": [{"type": "Feature",
               "properties": {"category": "purple"},
               "geometry": {"type": "Polygon", "coordinates": [[[75.8, 26.9], [75.9, 26.9], [75.9, 27.0], [75.8, 26.9]]]}}]}
        with self.assertRaises(ValueError):
            AP.zones_from_geojson(doc)

    def test_cache_marks_stale_on_outage_then_expires(self):
        now = [datetime(2026, 10, 26, tzinfo=timezone.utc)]

        class Flaky:
            up = True

            def fetch(self_inner):
                if not Flaky.up:
                    raise AP.AirspaceUnavailable("HTTP 503")
                s = AP.GeoJSONFileProvider(FIXTURE, "fixture").fetch()
                s.retrieved_at = now[0]
                return s

        cache = AP.CachedAirspaceProvider(Flaky(), max_age_s=600, clock=lambda: now[0])
        self.assertFalse(cache.fetch().stale)
        Flaky.up = False
        now[0] += timedelta(seconds=300)
        s = cache.fetch()
        self.assertTrue(s.stale)
        self.assertIn("503", s.error)
        now[0] += timedelta(seconds=400)
        with self.assertRaises(AP.AirspaceDataStale):
            cache.fetch()

    def test_cache_without_data_raises_unavailable(self):
        cache = AP.CachedAirspaceProvider(AP.GeoJSONFileProvider("/nonexistent.geojson", "x"))
        with self.assertRaises(AP.AirspaceUnavailable):
            cache.fetch()

    def test_scenario_provider_is_not_official(self):
        snap = AP.ScenarioProvider(JAIPUR_DISASTER).fetch()
        self.assertFalse(snap.official)
        self.assertTrue(all(z.category == "scenario" for z in snap.zones))
        self.assertEqual(len(snap.planning_polygons()), len(JAIPUR_DISASTER.no_fly_zones))

    def test_digitalsky_checker_unconfigured_reports_unchecked(self):
        r = AP.DigitalSkyIntersectionChecker(transport=None, token=None).check([(26.9, 75.8)] * 3)
        self.assertFalse(r.checked)
        self.assertIsNone(r.intersects)
        self.assertIn("not configured", r.detail)

    def test_digitalsky_checker_with_transport(self):
        r = AP.DigitalSkyIntersectionChecker(lambda **kw: {"intersects": True}, token="t").check(
            [(26.9, 75.8), (26.9, 75.9), (27.0, 75.9)])
        self.assertTrue(r.checked)
        self.assertTrue(r.intersects)
        r = AP.DigitalSkyIntersectionChecker(lambda **kw: {"weird": 1}, token="t").check([(0, 0)] * 3)
        self.assertFalse(r.checked)

        def boom(**kw):
            raise OSError("timeout")
        r = AP.DigitalSkyIntersectionChecker(boom, token="t").check([(0, 0)] * 3)
        self.assertFalse(r.checked)
        self.assertIn("timeout", r.detail)


class TestMissionRunnerObstacleAvoidance(unittest.TestCase):
    def test_runner_commands_hold_on_obstacle_conflict(self):
        from flightstack.mission_runner import MissionRunner
        from flightstack.obstacle_tracker import ObstacleTracker
        c = JAIPUR_DISASTER
        plan = build_delivery_mission((c.depot.lat, c.depot.lon), [c.deliveries[5]],
                                      [list(z) for z in c.no_fly_zones], MissionProfile())
        conn = FakeConn()
        link = AutopilotLink(conn)
        link.start()
        try:
            tracker = ObstacleTracker(safety_margin_m=30.0, time_horizon_s=12.0)
            runner = MissionRunner(link, plan, c.no_fly_zones, MissionProfile(), (c.depot.lat, c.depot.lon),
                                   obstacle_tracker=tracker)
            runner.state.took_off = True

            # Inject telemetry where drone is flying north at 10 m/s (vx=1000 cm/s)
            link.telemetry.update(Msg("GLOBAL_POSITION_INT", lat=int(c.depot.lat * 1e7),
                                      lon=int(c.depot.lon * 1e7), alt=491000, relative_alt=60000,
                                      vx=1000, vy=0, vz=0, hdg=0, time_boot_ms=1000))
            link.telemetry.update(Msg("HEARTBEAT", custom_mode=3, base_mode=128))  # AUTO mode, armed
            link.telemetry.update(Msg("SYS_STATUS", battery_remaining=90, voltage_battery=24000, current_battery=1000))

            # Register obstacle directly ahead at 40m distance, stationary -> conflict alert
            tracker.update_track("CRITICAL_CRANE", pos_ned_m=(40.0, 0.0, -60.0), vel_ned_ms=(0.0, 0.0, 0.0))

            # Step runner
            runner.step()

            # Mode switch to LOITER (mode 5 in ArduCopter, MAV_CMD_DO_SET_MODE = 176) commanded to fake connection
            self.assertTrue(len(conn.mav.commands) > 0)
            cmd_id, params = conn.mav.commands[-1]
            self.assertEqual(cmd_id, 176)
            self.assertEqual(params[1], 5)  # LOITER mode enum
        finally:
            link.close()


class TestAirspace4DValidation(unittest.TestCase):
    def test_altitude_separation_avoids_conflict(self):
        from airspace_provider import AirspaceZone
        # Zone on the segment path, but vertical floor is 100m AGL (drone flies at 60m AGL)
        zone_high = AirspaceZone(
            zone_id="HIGH_CRANE",
            category="temporary_restriction",
            polygon=[(26.85, 75.80), (26.85, 75.82), (26.87, 75.82), (26.87, 75.80)],
            floor_m_agl=100.0,
            ceiling_m_agl=200.0,
        )
        points = [(26.84, 75.81), (26.88, 75.81)]
        # Drone at 60m AGL: should NOT raise conflict because it's below floor
        validate_polyline(points, [zone_high], margin_m=0.0, alt_m_agl=60.0)

    def test_altitude_penetration_raises_conflict(self):
        from airspace_provider import AirspaceZone
        zone_mid = AirspaceZone(
            zone_id="MID_TFR",
            category="temporary_restriction",
            polygon=[(26.85, 75.80), (26.85, 75.82), (26.87, 75.82), (26.87, 75.80)],
            floor_m_agl=30.0,
            ceiling_m_agl=100.0,
        )
        points = [(26.84, 75.81), (26.88, 75.81)]
        # Drone at 60m AGL penetrates [30m, 100m]: MUST raise conflict
        with self.assertRaises(MissionValidationError):
            validate_polyline(points, [zone_mid], margin_m=0.0, alt_m_agl=60.0)

    def test_time_window_filtering(self):
        from datetime import datetime, timezone, timedelta
        from airspace_provider import AirspaceZone
        now = datetime.now(timezone.utc)
        # TFR only active in 3 hours
        future_tfr = AirspaceZone(
            zone_id="FUTURE_VIP_VISIT",
            category="temporary_restriction",
            polygon=[(26.85, 75.80), (26.85, 75.82), (26.87, 75.82), (26.87, 75.80)],
            valid_from=now + timedelta(hours=3),
            valid_to=now + timedelta(hours=5),
        )
        points = [(26.84, 75.81), (26.88, 75.81)]
        # Flight happening now: should NOT raise conflict
        validate_polyline(points, [future_tfr], margin_m=0.0, start_time=now, cruise_speed_ms=10.0)

        # Flight happening in 3.5 hours: MUST raise conflict
        with self.assertRaises(MissionValidationError):
            validate_polyline(points, [future_tfr], margin_m=0.0, start_time=now + timedelta(hours=3.5), cruise_speed_ms=10.0)

    def test_mission_runner_4d_airspace_update_time_and_altitude(self):
        from datetime import datetime, timezone, timedelta
        from airspace_provider import AirspaceZone
        from flightstack.mission_runner import MissionRunner, EventLog
        c = JAIPUR_DISASTER
        home = (c.depot.lat, c.depot.lon)
        stop = c.deliveries[5]
        plan = build_delivery_mission(home, [stop], [], MissionProfile(cruise_alt_m=50.0))
        conn = FakeConn()
        link = AutopilotLink(conn)
        link.start()
        try:
            log = EventLog(echo=False)
            runner = MissionRunner(link, plan, [], MissionProfile(cruise_alt_m=50.0), home, log=log)
            link.telemetry.update(Msg("GLOBAL_POSITION_INT", lat=int(home[0] * 1e7),
                                      lon=int(home[1] * 1e7), alt=480000, relative_alt=50000,
                                      vx=0, vy=0, vz=0, hdg=0, time_boot_ms=1000))
            link.telemetry.update(Msg("MISSION_CURRENT", seq=2))

            mid_lat = (home[0] + stop.lat) / 2.0
            mid_lon = (home[1] + stop.lon) / 2.0
            poly = [
                (mid_lat - 0.005, mid_lon - 0.005),
                (mid_lat - 0.005, mid_lon + 0.005),
                (mid_lat + 0.005, mid_lon + 0.005),
                (mid_lat + 0.005, mid_lon - 0.005),
            ]
            now = datetime.now(timezone.utc)
            # 1) High-altitude TFR (floor 90m > 50m cruise): no conflict
            high_zone = AirspaceZone("HIGH_TFR", "temporary_restriction", poly,
                                     floor_m_agl=90.0, ceiling_m_agl=150.0)
            res1 = runner.apply_airspace_update([high_zone], "high_alt_tfr", {"provider": "test"}, now=now)
            self.assertEqual(res1["event"], "airspace_update_no_conflict")

            # 2) Future TFR (active in +2h): no conflict now
            future_zone = AirspaceZone("TIMED_TFR", "temporary_restriction", poly,
                                       floor_m_agl=0.0, ceiling_m_agl=120.0,
                                       valid_from=now + timedelta(hours=2),
                                       valid_to=now + timedelta(hours=4))
            res2 = runner.apply_airspace_update([future_zone], "future_tfr", {"provider": "test"}, now=now)
            self.assertEqual(res2["event"], "airspace_update_no_conflict")

            # 3) Same TFR evaluated inside its active window (+2.5h): triggers conflict & replan
            res3 = runner.apply_airspace_update([future_zone], "active_tfr", {"provider": "test"},
                                                now=now + timedelta(hours=2, minutes=30))
            self.assertEqual(res3["reason"], "active_tfr")
            self.assertEqual(res3["version"], 1)
            self.assertGreater(res3["new_planned_km"], res3["old_planned_km"])
            self.assertEqual(len(runner.state.replans), 1)
            self.assertEqual(log.events[-1]["event"], "mission_replanned")
            self.assertEqual(len(conn.mav.items), len(runner.plan.items))
        finally:
            link.close()


class TestDeliveryStateMachine(unittest.TestCase):
    def test_separated_delivery_lifecycle(self):
        from flightstack.payload import MedicalPackage, PayloadManager, PackageStatus, DeliveryState, ReleaseSource
        pkg = MedicalPackage(package_id="PKG-01", assigned_stop=1, mass_kg=2.5)
        pm = PayloadManager([pkg])
        self.assertEqual(pkg.delivery_state, DeliveryState.LOADED)
        self.assertEqual(pkg.status, PackageStatus.LOADED)

        # 1. Arrival
        ok, _, _ = pm.record_arrival(1, 45.0)
        self.assertTrue(ok)
        self.assertEqual(pkg.delivery_state, DeliveryState.ARRIVED)
        self.assertEqual(pkg.arrived_time_s, 45.0)

        # 2. Site assessment
        ok, mode, _ = pm.assess_site(1, None, "hover", 46.0)
        self.assertTrue(ok)
        self.assertEqual(pkg.delivery_state, DeliveryState.METHOD_SELECTED)

        # 3. Release commanded
        ok, _, _ = pm.command_release(1, "hover", 50.0)
        self.assertTrue(ok)
        self.assertEqual(pkg.delivery_state, DeliveryState.RELEASE_COMMANDED)
        self.assertEqual(pkg.status, PackageStatus.RELEASE_COMMANDED)

        # 4. Actuator acknowledged
        ok, _, _ = pm.acknowledge_actuator(1, 51.0, ReleaseSource.SIM_GRIPPER_FEEDBACK)
        self.assertTrue(ok)
        self.assertEqual(pkg.delivery_state, DeliveryState.RELEASE_ACKNOWLEDGED)

        # 5. Custody confirmed
        ok, _, _ = pm.confirm_custody(1, 53.0, ReleaseSource.HOVER_ROPE_RELEASE)
        self.assertTrue(ok)
        self.assertEqual(pkg.delivery_state, DeliveryState.CUSTODY_CONFIRMED)

        # 6. Delivery complete
        ok, _, _ = pm.complete_delivery(1, 54.0)
        self.assertTrue(ok)
        self.assertEqual(pkg.delivery_state, DeliveryState.DELIVERY_COMPLETE)
        self.assertEqual(pkg.status, PackageStatus.DELIVERED)
        self.assertEqual(pm.remaining_cargo_kg(), 0.0)

    def test_site_assessment_downgrades_flooded_land_site(self):
        from flightstack.payload import MedicalPackage, PayloadManager, SiteSuitability
        pkg = MedicalPackage(package_id="PKG-FLOOD", assigned_stop=2, mass_kg=2.0)
        pm = PayloadManager([pkg])
        pm.record_arrival(2, 30.0)

        # Land delivery requested, but ground is flooded
        site = SiteSuitability(surface_condition="flooded", obstacle_clearance_m=12.0)
        ok, selected_mode, reason = pm.assess_site(2, site, "land", 32.0)
        self.assertTrue(ok)
        self.assertEqual(selected_mode, "hover")
        self.assertIn("Degraded from land to hover", reason)

    def test_land_mode_uses_nav_delay_for_continue_after_land(self):
        from flightstack.mission import build_delivery_mission, MissionProfile, MAV_CMD_NAV_DELAY
        c = JAIPUR_DISASTER
        plan = build_delivery_mission((c.depot.lat, c.depot.lon), [c.deliveries[0]],
                                      [list(z) for z in c.no_fly_zones],
                                      MissionProfile(delivery_mode="land"))
        cmds = [it.command for it in plan.items]
        self.assertIn(MAV_CMD_NAV_DELAY, cmds)

    def test_single_uav_reference_scenario_and_site_suitability_modes(self):
        from config import JAIPUR_SINGLE_UAV
        from flightstack.mission import build_delivery_mission, MissionProfile
        from flightstack.vehicle_model import VehicleProfile, BatterySpec, EnergyEstimator
        c = JAIPUR_SINGLE_UAV
        self.assertEqual(c.drone.fleet_size, 1)
        self.assertEqual(c.drone.capacity_kg, 8.0)
        total_cargo = sum(s.demand_kg for s in c.deliveries)
        self.assertLessEqual(total_cargo, c.drone.capacity_kg)

        # When operator requests delivery_mode="land", Stop 1 (flooded) must auto-downgrade to hover
        # while Stop 2 (clear elevated courtyard) executes land
        plan = build_delivery_mission(
            (c.depot.lat, c.depot.lon),
            c.deliveries,
            [list(z) for z in c.no_fly_zones],
            MissionProfile(delivery_mode="land", cruise_speed_ms=10.0),
        )
        stop1_roles = [it.role for it in plan.items if it.stop_id == 1]
        stop2_roles = [it.role for it in plan.items if it.stop_id == 2]
        self.assertIn("hover_descend", stop1_roles)
        self.assertNotIn("land", stop1_roles)
        self.assertIn("land", stop2_roles)
        self.assertIn("ground_dwell", stop2_roles)

        # Verify physical energy feasibility against VehicleProfile + BatterySpec
        ee = EnergyEstimator(VehicleProfile(max_payload_kg=c.drone.capacity_kg),
                             BatterySpec(reserve_fraction=c.drone.reserve_fraction))
        rep = ee.evaluate_route(plan.planned_km, [(s, total_cargo) for s in plan.stops],
                                cruise_speed_ms=10.0, hover_time_s=5.0)
        self.assertTrue(rep.is_feasible)
        self.assertLess(rep.total_energy_wh, rep.usable_energy_wh)


class TestNativeObstacleAvoidanceProximity(unittest.TestCase):
    def test_obstacle_distance_streamed_to_mavlink(self):
        from flightstack.mission_runner import MissionRunner
        from flightstack.obstacle_tracker import ObstacleTracker
        c = JAIPUR_DISASTER
        plan = build_delivery_mission((c.depot.lat, c.depot.lon), [c.deliveries[5]],
                                      [list(z) for z in c.no_fly_zones], MissionProfile())
        conn = FakeConn()
        link = AutopilotLink(conn)
        link.start()
        try:
            tracker = ObstacleTracker(safety_margin_m=30.0)
            runner = MissionRunner(link, plan, c.no_fly_zones, MissionProfile(), (c.depot.lat, c.depot.lon),
                                   obstacle_tracker=tracker)
            runner.state.took_off = True

            # Drone heading North (0 deg)
            link.telemetry.update(Msg("GLOBAL_POSITION_INT", lat=int(c.depot.lat * 1e7),
                                      lon=int(c.depot.lon * 1e7), alt=491000, relative_alt=60000,
                                      vx=1000, vy=0, vz=0, hdg=0, time_boot_ms=1000))
            link.telemetry.update(Msg("HEARTBEAT", custom_mode=3, base_mode=128))
            link.telemetry.update(Msg("SYS_STATUS", battery_remaining=90, voltage_battery=24000, current_battery=1000))

            # Obstacle 15 meters directly ahead
            tracker.update_track("TREE_HAZARD", pos_ned_m=(15.0, 0.0, -60.0), vel_ned_ms=(0.0, 0.0, 0.0))

            # Step runner: triggers _stream_proximity_obstacle_distance
            runner.step()

            # Verify MAVLink obstacle_distance was sent
            self.assertTrue(hasattr(conn.mav, "obstacle_distances"))
            self.assertGreater(len(conn.mav.obstacle_distances), 0)
            time_us, distances, min_d, max_d = conn.mav.obstacle_distances[-1]
            self.assertEqual(len(distances), 72)
            # Sector 0 is 0 deg (North / straight ahead) -> should be ~1500 cm (15m)
            self.assertAlmostEqual(distances[0], 1500, delta=100)
        finally:
            link.close()


if __name__ == "__main__":
    unittest.main()

