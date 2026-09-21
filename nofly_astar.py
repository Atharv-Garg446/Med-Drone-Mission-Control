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
from typing import List, Optional, Tuple, Dict

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


_GRID_CACHE: Dict[Tuple, Tuple[List[List[Point]], List[List[bool]], int, int]] = {}


def clear_grid_cache():
    """Clear the cached A* obstacle grids."""
    _GRID_CACHE.clear()


def _build_grid(start: Point, end: Point, zones: List[Polygon],
                 cell_size_km: float, buffer_km: float):
    """Build the grid of (lat, lon) cell centers plus a blocked[] lookup.

    Returns (grid_points: 2D list[row][col] of (lat,lon), blocked: 2D list
    of bool, shape (rows, cols)).
    """
    margin_m = buffer_km * 1000.0

    # Bounding box over start, end, and every no-fly zone,
    # padded so the grid has enough room to route around obstacles.
    lats = [start[0], end[0]] + [p[0] for z in zones for p in z]
    lons = [start[1], end[1]] + [p[1] for z in zones for p in z]
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons), max(lons)

    pad_km = (margin_m / 1000.0) + max(cell_size_km * 4, 1.0)
    mid_lat = (min_lat + max_lat) / 2.0
    cos_mid = max(math.cos(math.radians(mid_lat)), 0.1)
    pad_lat = pad_km / 111.0
    pad_lon = pad_km / (111.0 * cos_mid)
    min_lat -= pad_lat; max_lat += pad_lat
    min_lon -= pad_lon; max_lon += pad_lon

    lat_step = cell_size_km / 111.0
    lon_step = cell_size_km / (111.0 * cos_mid)

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

    # CRITICAL: Account for the actual resulting grid step after MAX_CELLS coarsening!
    actual_cell_lat_km = lat_step * 111.0
    actual_cell_lon_km = lon_step * 111.0 * cos_mid
    actual_cell_diag_m = math.hypot(actual_cell_lat_km, actual_cell_lon_km) * 1000.0 / 2.0
    block_margin_m = margin_m + actual_cell_diag_m

    cache_key = (
        round(min_lat, 6), round(max_lat, 6),
        round(min_lon, 6), round(max_lon, 6),
        rows, cols, round(block_margin_m, 2),
        tuple(tuple(p) for z in zones for p in z)
    )
    if cache_key in _GRID_CACHE:
        return _GRID_CACHE[cache_key]

    pad_box_km = block_margin_m / 1000.0
    pad_box_lat = pad_box_km / 111.0
    pad_box_lon = pad_box_km / (111.0 * cos_mid)
    zone_boxes = []
    for z in zones:
        z_lats = [p[0] for p in z]
        z_lons = [p[1] for p in z]
        zone_boxes.append((
            min(z_lats) - pad_box_lat, max(z_lats) + pad_box_lat,
            min(z_lons) - pad_box_lon, max(z_lons) + pad_box_lon,
            z
        ))

    grid_points = [[(min_lat + r * lat_step, min_lon + c * lon_step)
                    for c in range(cols)] for r in range(rows)]

    def _is_cell_blocked(pt: Point) -> bool:
        lat, lon = pt
        for min_lt, max_lt, min_ln, max_ln, z in zone_boxes:
            if min_lt <= lat <= max_lt and min_ln <= lon <= max_ln:
                if point_near_polygon(pt, z, block_margin_m):
                    return True
        return False

    blocked = [[_is_cell_blocked(grid_points[r][c]) for c in range(cols)] for r in range(rows)]
    res = (grid_points, blocked, rows, cols)
    _GRID_CACHE[cache_key] = res
    return res


def _nearest_free_cell(grid_points, blocked, rows, cols, target: Point) -> Tuple[int, int]:
    """Snap an arbitrary lat/lon to the nearest grid cell that ISN'T blocked."""
    min_lat, min_lon = grid_points[0][0]
    lat_step = (grid_points[1][0][0] - min_lat) if rows > 1 else 0.001
    lon_step = (grid_points[0][1][1] - min_lon) if cols > 1 else 0.001

    tr = max(0, min(rows - 1, int(round((target[0] - min_lat) / lat_step))))
    tc = max(0, min(cols - 1, int(round((target[1] - min_lon) / lon_step))))

    if not blocked[tr][tc]:
        return (tr, tc)

    # Search outward in rings around (tr, tc)
    best = None
    best_dist = float("inf")
    max_radius = max(rows, cols)

    for radius in range(1, max_radius):
        found = False
        for dr in (-radius, radius):
            r = tr + dr
            if 0 <= r < rows:
                for c in range(max(0, tc - radius), min(cols, tc + radius + 1)):
                    if not blocked[r][c]:
                        d = haversine_km(grid_points[r][c], target)
                        if d < best_dist:
                            best_dist = d
                            best = (r, c)
                            found = True
        for dc in (-radius, radius):
            c = tc + dc
            if 0 <= c < cols:
                for r in range(max(0, tr - radius + 1), min(rows, tr + radius)):
                    if not blocked[r][c]:
                        d = haversine_km(grid_points[r][c], target)
                        if d < best_dist:
                            best_dist = d
                            best = (r, c)
                            found = True
        if found:
            return best

    raise RuntimeError("No-fly zone A*: entire local grid is blocked -- "
                        "increase grid size or shrink the buffer_km margin.")


def astar_around_obstacles(start: Point, end: Point, zones: List[Polygon],
                            cell_size_km: float = 0.25,
                            buffer_km: float = 0.3) -> Tuple[List[Point], float]:
    """Find the shortest start->end path that avoids every polygon in
    `zones`, using grid-based A*.

    Returns (waypoint_list, total_distance_km). waypoint_list always starts
    with `start` and ends with `end` exactly (the grid path is spliced onto
    the real endpoints so we don't lose precision snapping to grid cells).
    """
    margin_m = buffer_km * 1000.0
    eff_cell_size_km = min(cell_size_km, 0.08) if buffer_km <= 0.1 else cell_size_km
    grid_points, blocked, rows, cols = _build_grid(start, end, zones, eff_cell_size_km, buffer_km)

    start_rc = _nearest_free_cell(grid_points, blocked, rows, cols, start)
    end_rc = _nearest_free_cell(grid_points, blocked, rows, cols, end)

    min_lat, min_lon = grid_points[0][0]
    lat_step = (grid_points[1][0][0] - min_lat) if rows > 1 else 0.001
    lon_step = (grid_points[0][1][1] - min_lon) if cols > 1 else 0.001
    mid_lat = (min_lat + grid_points[-1][0][0]) * 0.5
    cos_mid = max(math.cos(math.radians(mid_lat)), 0.1)

    cell_lat_km = lat_step * 111.0
    cell_lon_km = lon_step * 111.0 * cos_mid
    cell_diag_km = math.hypot(cell_lat_km, cell_lon_km)

    neighbor_offsets = [
        (-1, -1, cell_diag_km), (-1, 0, cell_lat_km), (-1, 1, cell_diag_km),
        ( 0, -1, cell_lon_km),                        ( 0, 1, cell_lon_km),
        ( 1, -1, cell_diag_km), ( 1, 0, cell_lat_km), ( 1, 1, cell_diag_km),
    ]

    def heuristic(rc):
        return math.hypot((rc[0] - end_rc[0]) * cell_lat_km, (rc[1] - end_rc[1]) * cell_lon_km)

    open_set = [(heuristic(start_rc), 0, start_rc)]
    g_score = {start_rc: 0.0}
    came_from = {}
    visited = set()
    tie_breaker = 0

    while open_set:
        _, _, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)

        if current == end_rc:
            break

        r, c = current
        for dr, dc, step_cost in neighbor_offsets:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if blocked[nr][nc]:
                continue
            if dr != 0 and dc != 0:
                # Prevent diagonal corner-cutting across restricted zone boundaries
                if blocked[r + dr][c] or blocked[r][c + dc]:
                    continue
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

    # Path Smoothing (Bidirectional String Pulling)
    # The raw A* grid path contains staircase artifacts from discrete grid cells.
    # We evaluate both forward and backward line-of-sight string-pulling passes
    # to eliminate doglegs while strictly enforcing the safety buffer margin.
    if len(waypoints) > 2:
        # Forward pass: greedily find furthest visible waypoint from current
        fwd = [waypoints[0]]
        curr = 0
        while curr < len(waypoints) - 1:
            furthest_visible = curr + 1
            for i in range(curr + 2, len(waypoints)):
                if not path_intersects_any_zone(waypoints[curr], waypoints[i], zones, margin_m=margin_m):
                    furthest_visible = i
            fwd.append(waypoints[furthest_visible])
            curr = furthest_visible

        # Backward pass: greedily find earliest visible waypoint from goal
        curr = len(waypoints) - 1
        bwd = [waypoints[curr]]
        while curr > 0:
            earliest_visible = curr - 1
            for i in range(0, curr - 1):
                if not path_intersects_any_zone(waypoints[i], waypoints[curr], zones, margin_m=margin_m):
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

    # Verification check: ensure the computed detour strictly honors the safety buffer
    for i in range(len(waypoints) - 1):
        for zone in zones:
            if segment_intersects_polygon(waypoints[i], waypoints[i + 1], zone, margin_m=max(0.0, margin_m - 0.5)):
                # If the drone start position is physically inside a pop-up zone, the first
                # segment escapes the zone towards the nearest free cell.
                if i == 0 and point_near_polygon(start, zone, margin_m) and not point_near_polygon(end, zone, margin_m) and len(waypoints) > 2:
                    continue
                raise RuntimeError(
                    f"No-fly zone A*: the computed detour breaches the {margin_m:.0f}m safety buffer. "
                    "Rejecting path as infeasible."
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
    If A* cannot find a valid path (e.g. destination inside a restricted zone
    or completely obstructed corridor), returns (None, float('inf'), True)
    so upstream solvers recognize the edge as unreachable.
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
