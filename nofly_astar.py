"""
nofly_astar.py
==============
Grid-based A* pathfinding around restricted airspace polygons.

When a straight line between two delivery points intersects a no-fly zone,
this module lays a local grid over the area, marks cells inside the zone
as blocked, and finds the shortest collision-free path.

HOW THE GRID WORKS
------------------
1. Take the bounding box of [start, end, every no-fly zone], padded a bit.
2. Chop that box into a grid of small square cells (cell_size_km wide).
3. A cell is "blocked" if its *center point* falls inside any no-fly
   polygon (checked with geo_utils.point_in_polygon), OR within a small
   safety buffer of one -- real drones shouldn't hug a restricted
   boundary exactly, so we inflate each polygon slightly first.
4. A* searches this grid like a maze: each cell can move to its 8
   neighbors (including diagonals), the path cost is real-world distance
   (haversine) between cell centers, and the heuristic is straight-line
   haversine distance to the goal (which never overestimates the true
   remaining cost -- required for A* to guarantee the shortest path).
"""

import heapq
import math
from typing import List, Optional, Tuple

from geo_utils import haversine_km, point_in_polygon, polygon_bounds, path_intersects_any_zone, segment_intersects_polygon, point_near_polygon

Point = Tuple[float, float]
Polygon = List[Point]


def _inflate_polygon(polygon: Polygon, buffer_km: float) -> Polygon:
    """Push every vertex outward from the polygon's centroid by roughly
    `buffer_km`. This is a deliberately simple safety margin (not a true
    geometric offset/buffer operation like shapely's .buffer()) -- good
    enough for a grid this coarse, and keeps this module dependency-free.
    """
    if buffer_km <= 0:
        return polygon
    centroid_lat = sum(p[0] for p in polygon) / len(polygon)
    centroid_lon = sum(p[1] for p in polygon) / len(polygon)

    inflated = []
    for lat, lon in polygon:
        dist = haversine_km((centroid_lat, centroid_lon), (lat, lon))
        if dist < 1e-9:
            inflated.append((lat, lon))
            continue
        # Move this vertex further from the centroid by buffer_km, along
        # the same lat/lon direction it's already in.
        scale = (dist + buffer_km) / dist
        new_lat = centroid_lat + (lat - centroid_lat) * scale
        new_lon = centroid_lon + (lon - centroid_lon) * scale
        inflated.append((new_lat, new_lon))
    return inflated


def _build_grid(start: Point, end: Point, zones: List[Polygon],
                 cell_size_km: float, buffer_km: float):
    """Build the grid of (lat, lon) cell centers plus a blocked[] lookup.

    Returns (grid_points: 2D list[row][col] of (lat,lon), blocked: 2D list
    of bool, shape (rows, cols)).
    """
    inflated_zones = [_inflate_polygon(z, buffer_km) for z in zones]

    # Bounding box over start, end, and every (inflated) no-fly zone,
    # so the grid has enough room to route around the obstacle.
    lats = [start[0], end[0]]
    lons = [start[1], end[1]]
    for zone in inflated_zones:
        for lat, lon in zone:
            lats.append(lat)
            lons.append(lon)
    min_lat, min_lon, max_lat, max_lon = min(lats), min(lons), max(lats), max(lons)

    # Small extra margin so the path isn't forced to hug the bbox edge.
    pad_lat = cell_size_km * 3 / 111.0
    pad_lon = cell_size_km * 3 / (111.0 * max(math.cos(math.radians((min_lat + max_lat) / 2)), 0.1))
    min_lat -= pad_lat; max_lat += pad_lat
    min_lon -= pad_lon; max_lon += pad_lon

    lat_step = cell_size_km / 111.0
    lon_step = cell_size_km / (111.0 * max(math.cos(math.radians((min_lat + max_lat) / 2)), 0.1))

    rows = max(3, int((max_lat - min_lat) / lat_step) + 1)
    cols = max(3, int((max_lon - min_lon) / lon_step) + 1)

    # Cap grid size so a huge no-fly zone can't make this crawl forever.
    MAX_CELLS = 120
    if rows > MAX_CELLS:
        lat_step *= rows / MAX_CELLS
        rows = MAX_CELLS
    if cols > MAX_CELLS:
        lon_step *= cols / MAX_CELLS
        cols = MAX_CELLS

    grid_points = [[(min_lat + r * lat_step, min_lon + c * lon_step)
                    for c in range(cols)] for r in range(rows)]

    blocked = [[any(point_in_polygon(grid_points[r][c], z) for z in inflated_zones)
                for c in range(cols)] for r in range(rows)]

    return grid_points, blocked, rows, cols


def _nearest_free_cell(grid_points, blocked, rows, cols, target: Point) -> Tuple[int, int]:
    """Snap an arbitrary lat/lon to the nearest grid cell that ISN'T blocked."""
    best = None
    best_dist = float("inf")
    for r in range(rows):
        for c in range(cols):
            if blocked[r][c]:
                continue
            d = haversine_km(grid_points[r][c], target)
            if d < best_dist:
                best_dist = d
                best = (r, c)
    if best is None:
        raise RuntimeError("No-fly zone A*: entire local grid is blocked -- "
                            "increase grid size or shrink the buffer_km margin.")
    return best


def astar_around_obstacles(start: Point, end: Point, zones: List[Polygon],
                            cell_size_km: float = 0.25,
                            buffer_km: float = 0.3) -> Tuple[List[Point], float]:
    """Find the shortest start->end path that avoids every polygon in
    `zones`, using grid-based A*.

    Returns (waypoint_list, total_distance_km). waypoint_list always starts
    with `start` and ends with `end` exactly (the grid path is spliced onto
    the real endpoints so we don't lose precision snapping to grid cells).
    """
    grid_points, blocked, rows, cols = _build_grid(start, end, zones, cell_size_km, buffer_km)

    start_rc = _nearest_free_cell(grid_points, blocked, rows, cols, start)
    end_rc = _nearest_free_cell(grid_points, blocked, rows, cols, end)

    # --- Standard A* search over the grid -----------------------------
    # open_set: min-heap of (f_score, tie_breaker, (r, c))
    # g_score:  best known real cost from start to this cell
    # came_from: for path reconstruction
    def heuristic(rc):
        return haversine_km(grid_points[rc[0]][rc[1]], grid_points[end_rc[0]][end_rc[1]])

    open_set = [(heuristic(start_rc), 0, start_rc)]
    g_score = {start_rc: 0.0}
    came_from = {}
    visited = set()
    tie_breaker = 0

    neighbor_offsets = [(-1, -1), (-1, 0), (-1, 1),
                         (0, -1),           (0, 1),
                         (1, -1),  (1, 0),  (1, 1)]

    while open_set:
        _, _, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)

        if current == end_rc:
            break

        r, c = current
        for dr, dc in neighbor_offsets:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if blocked[nr][nc]:
                continue
            if dr != 0 and dc != 0:
                # Prevent diagonal corner-cutting across restricted zone boundaries
                if blocked[r + dr][c] or blocked[r][c + dc]:
                    continue
            step_cost = haversine_km(grid_points[r][c], grid_points[nr][nc])
            tentative_g = g_score[current] + step_cost
            neighbor = (nr, nc)
            if tentative_g < g_score.get(neighbor, float("inf")):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                tie_breaker += 1
                heapq.heappush(open_set, (tentative_g + heuristic(neighbor), tie_breaker, neighbor))

    if end_rc not in came_from and end_rc != start_rc:
        raise RuntimeError("No-fly zone A*: no path found around the obstacle on this grid. "
                            "Try a smaller cell_size_km for finer resolution.")

    # Reconstruct path of grid cells, then convert to lat/lon waypoints
    path_cells = [end_rc]
    while path_cells[-1] != start_rc:
        path_cells.append(came_from[path_cells[-1]])
    path_cells.reverse()

    waypoints = [start] + [grid_points[r][c] for (r, c) in path_cells[1:-1]] + [end]

    margin_m = buffer_km * 1000.0

    # Path Smoothing (Bidirectional String Pulling)
    # The raw A* grid path contains staircase artifacts from discrete grid cells.
    # We evaluate both forward and backward line-of-sight string-pulling passes
    # to eliminate doglegs and overshoot, choosing whichever minimizes flight distance.
    if len(waypoints) > 2:
        # Forward pass: greedily find furthest visible waypoint from current
        fwd = [waypoints[0]]
        curr = 0
        while curr < len(waypoints) - 1:
            furthest_visible = curr + 1
            for i in range(curr + 2, len(waypoints)):
                if not path_intersects_any_zone(waypoints[curr], waypoints[i], zones, margin_m=0.0):
                    furthest_visible = i
            fwd.append(waypoints[furthest_visible])
            curr = furthest_visible

        # Backward pass: greedily find earliest visible waypoint from goal
        curr = len(waypoints) - 1
        bwd = [waypoints[curr]]
        while curr > 0:
            earliest_visible = curr - 1
            for i in range(0, curr - 1):
                if not path_intersects_any_zone(waypoints[i], waypoints[curr], zones, margin_m=0.0):
                    earliest_visible = i
                    break
            bwd.append(waypoints[earliest_visible])
            curr = earliest_visible
        bwd.reverse()

        d_fwd = sum(haversine_km(fwd[i], fwd[i + 1]) for i in range(len(fwd) - 1))
        d_bwd = sum(haversine_km(bwd[i], bwd[i + 1]) for i in range(len(bwd) - 1))
        waypoints = bwd if d_bwd <= d_fwd else fwd

    for zone in zones:
        if point_near_polygon(end, zone, margin_m):
            raise RuntimeError(
                f"No-fly zone A*: destination ({end[0]:.4f}, {end[1]:.4f}) lies inside or within "
                f"{margin_m:.0f}m safety buffer of restricted zone."
            )

    # Verification check: ensure the computed detour does not clip restricted airspace
    for i in range(len(waypoints) - 1):
        for zone in zones:
            if segment_intersects_polygon(waypoints[i], waypoints[i + 1], zone):
                # If the drone start position is physically inside a pop-up zone, the first
                # segment escapes the zone towards the nearest free cell. Only subsequent
                # segments or clipping other zones is strictly forbidden.
                if i == 0 and point_near_polygon(start, zone, margin_m) and not point_near_polygon(end, zone, margin_m) and len(waypoints) > 2:
                    continue
                raise RuntimeError(
                    "No-fly zone A*: the computed detour still clips a restricted "
                    "zone. This usually means the zone is far larger than the local "
                    "search grid (both endpoints snapped to the same escape cell). "
                    "Increase the grid's implicit search radius or cell_size_km."
                )


    total_km = 0.0
    for i in range(len(waypoints) - 1):
        total_km += haversine_km(waypoints[i], waypoints[i + 1])

    return waypoints, total_km


def route_avoiding_zones(p1: Point, p2: Point, zones: List[Polygon],
                          cell_size_km: float = 0.25,
                          buffer_km: float = 0.3) -> Tuple[Optional[List[Point]], float, bool]:
    """Top-level helper used by distance_matrix.py: if the direct p1->p2
    segment doesn't cross any no-fly zone, return it unchanged (fast path,
    no grid search needed). If it does, run A* around the obstacle(s).

    Returns (waypoints, distance_km, was_rerouted).

    If A* fails to find a valid path (too many overlapping zones, endpoint
    inside a zone, etc.), we fall back to a large penalty distance rather
    than crashing the entire pipeline.  This is essential for robustness
    in random simulations where dynamically injected TFRs may create
    unsolvable airspace geometry.
    """
    margin_m = buffer_km * 1000.0

    # If the destination lies inside or within the safety buffer of any restricted zone,
    # the destination is temporarily unreachable.
    for zone in zones:
        if point_near_polygon(p2, zone, margin_m):
            return None, float('inf'), True

    if not zones or not path_intersects_any_zone(p1, p2, zones, margin_m=margin_m):
        return [p1, p2], haversine_km(p1, p2), False

    try:
        waypoints, distance_km = astar_around_obstacles(p1, p2, zones, cell_size_km, buffer_km)
        return waypoints, distance_km, True
    except (RuntimeError, KeyError):
        # A* failed — the TFR(s) created an unsolvable airspace geometry.
        # We must NOT fall back to an illegal straight-line path. We return
        # infinity distance and no path, so the router knows this edge is dead.
        return None, float('inf'), True
