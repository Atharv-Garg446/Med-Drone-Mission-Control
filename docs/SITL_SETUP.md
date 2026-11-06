# ArduPilot SITL setup (verified)

The flight-stack demonstrations use the **real ArduCopter firmware** compiled for
the host machine (Software-In-The-Loop), talking MAVLink to this repository's
planner. Flight control, EKF state estimation, failsafes and the gripper
driver all run inside ArduPilot; this repository does not implement any of them.

## Verified environment

| Item | Version |
|---|---|
| Host | macOS 26.2, Apple Silicon (arm64), 8 GB RAM |
| Compiler | Apple clang 21 (Xcode Command Line Tools) |
| ArduPilot | `Copter-4.5` branch, built with `./waf configure --board sitl && ./waf copter` (9 min) |
| Physics | ArduPilot built-in multicopter model (`--model +`) |
| Python link | pymavlink 2.4.50 in a separate virtualenv |

Docker and Gazebo were **not** installed on this machine. Gazebo (Harmonic) on
macOS arm64 is not a Tier-1 platform; the camera/obstacle work in Phase 3 is
planned for Ubuntu 22.04/24.04 (native or VM) with `ardupilot_gazebo`, where
ArduPilot connects through `--model JSON`. The planner side does not change.

## Build

```bash
# next to this repository (flightstack/sitl.py also finds ../ardupilot)
git clone --depth 1 --branch Copter-4.5 --recurse-submodules --shallow-submodules \
    https://github.com/ArduPilot/ardupilot.git ../ardupilot
python3 -m venv ../.venv-sitl
../.venv-sitl/bin/pip install -r requirements-sitl.txt
source ../.venv-sitl/bin/activate
(cd ../ardupilot && ./waf configure --board sitl && ./waf copter)
```

The binary is `../ardupilot/build/sitl/bin/arducopter`. You can override discovery with
`ARDUPILOT_HOME` or `ARDUCOPTER_SITL_BIN`.

## Run

```bash
# Planner -> mission -> SITL flight, hover-and-release delivery
../.venv-sitl/bin/python sitl_mission_demo.py

# Mid-flight airspace change -> conflict detected -> LOITER -> re-plan -> resume
../.venv-sitl/bin/python sitl_mission_demo.py --airspace-change

# Land-and-deliver mode
../.venv-sitl/bin/python sitl_mission_demo.py --mode land --route 2 --max-stops 1

# Opt-in integration test (skipped automatically in CI)
../.venv-sitl/bin/python -m unittest tests.test_sitl_integration -v
```

Each run writes `output/sitl_run_<timestamp>.jsonl` (all decisions plus 1 Hz
autopilot telemetry) and `output/sitl_mission_<timestamp>.waypoints`
(QGC WPL 110, loadable in Mission Planner / QGroundControl).

To watch the flight, connect QGroundControl or MAVProxy to the second SITL
serial port: `mavproxy.py --master tcp:127.0.0.1:5762 --map`.

## Gotchas found during verification

* Uploading a mission immediately after the first heartbeat is rejected with
  `MAV_MISSION_NO_SPACE` ("Only 0 items are supported") because mission storage
  is still initialising. `AutopilotLink.wait_ready()` waits for a healthy EKF
  position and a 3D GNSS fix first.
* SITL's TCP port 5760 accepts one client. Use 5762/5763 for a GCS.
* `flightstack/params/sitl_delivery.parm` enables a simulated servo gripper
  (`SIM_GRPS_*`) and a finite simulated battery (`SIM_BATT_CAP_AH=10`). These are
  SITL settings, **not** a calibrated model of any real airframe.
