"""
flightstack: integration layer between the mission planner in this repository
and an ArduPilot autopilot (SITL or hardware) over MAVLink.

  telemetry.py     autopilot-reported state, freshness/validity checks
  mission.py       planner route -> MAVLink mission items (+ airspace validation)
  mavlink_link.py  pymavlink client: telemetry pump, mission upload, commands
  sitl.py          ArduCopter SITL process manager
  mission_runner.py  executes a plan, logs events, applies airspace updates
"""
