"""
SITL integration test: real ArduCopter firmware over MAVLink.

Skipped automatically unless pymavlink is installed AND an ArduCopter SITL
binary is found (ARDUPILOT_HOME / ARDUCOPTER_SITL_BIN, see docs/simulation_setup.md).
Run explicitly with:
    ARDUPILOT_HOME=../ardupilot ../.venv-sitl/bin/python -m unittest tests.test_sitl_integration -v

Acceptance checks (Phase 1, milestone M1):
  * autopilot heartbeat + EKF/GNSS-healthy position telemetry received
  * mission uploaded and read back with identical item count / coordinates
  * vehicle arms, takes off in AUTO and reaches the first mission waypoint
  * killing the autopilot makes telemetry go stale and the planner refuses
    to use it
"""

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from flightstack.sitl import find_sitl_binary, SitlProcess
from flightstack.telemetry import StaleTelemetryError

try:
    import pymavlink  # noqa: F401
    HAVE_MAV = True
except ImportError:
    HAVE_MAV = False

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@unittest.skipUnless(HAVE_MAV and find_sitl_binary(), "pymavlink and ArduCopter SITL binary required")
class TestSitlIntegration(unittest.TestCase):
    def test_upload_fly_and_stale_detection(self):
        from config import JAIPUR_DISASTER as c
        from flightstack.mavlink_link import AutopilotLink
        from flightstack.mission import MissionProfile, build_delivery_mission

        home = (c.depot.lat, c.depot.lon)
        sitl = SitlProcess(home=(home[0], home[1], 431, 0), speedup=5, instance=3,
                           extra_defaults=[os.path.join(ROOT, "flightstack", "params",
                                                        "sitl_delivery.parm")]).start()
        link = None
        try:
            link = AutopilotLink.connect(sitl.url)
            link.wait_ready(120)
            pos = link.telemetry.fresh_position()
            self.assertAlmostEqual(pos.lat, home[0], places=3)
            self.assertAlmostEqual(pos.lon, home[1], places=3)

            plan = build_delivery_mission(home, [c.deliveries[7]], [list(z) for z in c.no_fly_zones],
                                          MissionProfile(cruise_alt_m=30))
            link.upload_mission(plan.items)
            back = link.download_mission()
            self.assertEqual(len(back), len(plan.items))
            for (seq, cmd, lat, lon, alt), it in zip(back[1:], plan.items[1:]):
                self.assertEqual(cmd, it.command)
                if it.lat:
                    self.assertAlmostEqual(lat, it.lat, places=6)

            link.set_mode("GUIDED")
            link.arm()
            link.set_mode("AUTO")
            link.start_mission()
            link.wait_for(lambda t: t.position.value and t.position.value.alt_rel_m > 25, 120,
                          "climb above 25 m")
            link.wait_for(lambda t: (t.mission_current.value or 0) >= 3, 180, "reach first waypoint")
            self.assertEqual(link.telemetry.mode, "AUTO")

            sitl.stop()                      # autopilot disappears
            time.sleep(3.0)
            with self.assertRaises(StaleTelemetryError):
                link.telemetry.fresh_position(max_age_s=2.0)
            self.assertFalse(link.telemetry.link_alive(2.0))
        finally:
            if link:
                link.close()
            sitl.stop()


if __name__ == "__main__":
    unittest.main()
