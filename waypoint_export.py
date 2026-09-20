"""
waypoint_export.py
===================
Exports a solved route as a real "QGC WPL 110" waypoint file -- the plain
text mission format used by QGroundControl and Mission Planner, and
understood natively by ArduPilot- and PX4-based flight controllers over
MAVLink.

>>> COMPATIBILITY, IN PLAIN TERMS <<<
This format flies on ArduCopter, PX4, and anything else speaking MAVLink
mission-protocol. It does NOT fly on DJI, Skydio, or other closed consumer
drone ecosystems -- those use proprietary app-based mission formats with no
public "load this waypoint file" path. See README "Scope & Design
Boundaries" for the full statement; this comment is the short version.

FILE FORMAT (tab-separated columns, one waypoint per line):

    INDEX  CURRENT  FRAME  COMMAND  PARAM1  PARAM2  PARAM3  PARAM4  LAT  LON  ALT  AUTOCONTINUE

  INDEX        0-based sequence number of this waypoint in the mission
  CURRENT      1 if this is the waypoint the vehicle starts the mission
               at (always the home/depot row, index 0), else 0
  FRAME        coordinate frame: 0 = MAV_FRAME_GLOBAL (absolute altitude,
               used only for the home row), 3 = MAV_FRAME_GLOBAL_RELATIVE_ALT
               (altitude relative to home -- used for every flight waypoint)
  COMMAND      MAVLink MAV_CMD id: 16 = NAV_WAYPOINT, 22 = NAV_TAKEOFF,
               21 = NAV_LAND
  PARAM1-4     command-specific parameters (mostly 0 for a plain waypoint;
               NAV_TAKEOFF uses PARAM1 to constrain climb angle when >0,
               we leave it at 0 meaning "no constraint")
  LAT, LON     decimal degrees
  ALT          meters -- absolute AMSL for the home row, relative-to-home
               for every other row (see FRAME above)
  AUTOCONTINUE 1 = keep flying the mission automatically after this
               waypoint, 0 = pause and wait (we always use 1)

Reference: https://mavlink.io/en/file_formats/#mission_plain_text_file
"""

import math
from typing import List, Optional
from config import Location, DroneConfig
from vrp_scratch import is_route_feasible

# MAV_CMD ids we use (see mavlink.io/en/messages/common.html for the full list)
CMD_NAV_WAYPOINT = 16
CMD_NAV_TAKEOFF = 22
CMD_NAV_LAND = 21

FRAME_GLOBAL_ABS = 0
FRAME_GLOBAL_RELATIVE_ALT = 3


def export_route_to_wpl(
    route: List[int],
    locations: List[Location],
    filepath: str,
    cruise_altitude_m: float = 50.0,
    detours: dict = None,
    matrix: Optional[List[List[float]]] = None,
    drone: Optional[DroneConfig] = None,
) -> str:
    """Write one solved route (depot -> ... -> depot) to a QGC WPL 110 file.

    Raises ValueError and refuses to export if the route is infeasible or
    contains unreachable edges.
    """
    if detours is None:
        detours = {}

    # Strict Fail Closed validation: Require matrix and drone configuration for export
    if matrix is None or drone is None:
        raise ValueError(
            f"Refusing to export route {route}: export requires both distance matrix and drone "
            f"configuration for strict feasibility validation."
        )

    if not is_route_feasible(route, locations, matrix, drone):
        raise ValueError(
            f"Refusing to export route {route}: Route violates feasibility constraints "
            f"(capacity, range, cold-chain, time-window, or unreachable edge)."
        )

    depot = locations[route[0]]
    lines = ["QGC WPL 110"]

    def wpl_line(seq, current, frame, cmd, p1, p2, p3, p4, lat, lon, alt, autocontinue=1):
        return "\t".join(str(v) for v in
                          [seq, current, frame, cmd, p1, p2, p3, p4, lat, lon, alt, autocontinue])

    # seq 0: home row -- always the depot, always absolute altitude, always "current"
    lines.append(wpl_line(0, 1, FRAME_GLOBAL_ABS, CMD_NAV_WAYPOINT,
                           0, 0, 0, 0, depot.lat, depot.lon, 0))

    # seq 1: takeoff to cruise altitude
    lines.append(wpl_line(1, 0, FRAME_GLOBAL_RELATIVE_ALT, CMD_NAV_TAKEOFF,
                           0, 0, 0, 0, depot.lat, depot.lon, cruise_altitude_m))

    seq = 2
    for i in range(len(route) - 1):
        u = route[i]
        v = route[i + 1]

        # If there's a detour for this leg (A* routed around a no-fly zone),
        # insert its intermediate waypoints BETWEEN the two delivery stops.
        # waypts[0] is the start point (already covered) and waypts[-1] is
        # the end point (will be added as the delivery stop below), so we
        # only insert waypts[1:-1] — the actual detour path.
        if (u, v) in detours:
            waypts = detours[(u, v)]
            for wp in waypts[1:-1]:
                lines.append(wpl_line(seq, 0, FRAME_GLOBAL_RELATIVE_ALT, CMD_NAV_WAYPOINT,
                                       0, 0, 0, 0, wp[0], wp[1], cruise_altitude_m))
                seq += 1

        # Add the destination stop of this leg (unless it's the final depot
        # return, which is handled by NAV_LAND below)
        if i < len(route) - 2:
            loc = locations[v]
            lines.append(wpl_line(seq, 0, FRAME_GLOBAL_RELATIVE_ALT, CMD_NAV_WAYPOINT,
                                   0, 0, 0, 0, loc.lat, loc.lon, cruise_altitude_m))
            seq += 1

    # final row: land back at the depot
    lines.append(wpl_line(seq, 0, FRAME_GLOBAL_RELATIVE_ALT, CMD_NAV_LAND,
                           0, 0, 0, 0, depot.lat, depot.lon, 0))

    with open(filepath, "w") as f:
        f.write("\n".join(lines) + "\n")

    return filepath


def export_all_routes(routes: List[List[int]], locations: List[Location],
                       output_dir: str = "output", cruise_altitude_m: float = 50.0,
                       detours: dict = None, matrix: Optional[List[List[float]]] = None,
                       drone: Optional[DroneConfig] = None) -> List[str]:
    """Export every route in a solution as its own numbered .waypoints file
    (one file per drone trip -- QGC WPL 110 doesn't have a native concept
    of "multiple missions in one file"). Returns the list of filepaths written."""
    import os
    os.makedirs(output_dir, exist_ok=True)
    paths = []
    for i, route in enumerate(routes):
        filepath = os.path.join(output_dir, f"route_{i + 1}.waypoints")
        export_route_to_wpl(route, locations, filepath, cruise_altitude_m, detours, matrix, drone)
        paths.append(filepath)
    return paths
