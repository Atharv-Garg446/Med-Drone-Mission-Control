"""
geo_utils.py
============
Geometry helpers used throughout the project:
  - haversine_km:            great-circle distance between two lat/lon points
  - point_in_polygon:        is a lat/lon point inside a polygon?
  - segment_intersects_polygon: does a straight line between two points cross a polygon?
  - polygon_bounds:          bounding box of a polygon

Implemented without external spatial libraries (shapely, geopandas) to minimize dependencies.
"""

import math
from typing import List, Tuple

Point = Tuple[float, float]     # (lat, lon)
Polygon = List[Point]


def haversine_km(p1: Point, p2: Point) -> float:
    """Great-circle distance between two (lat, lon) points, in kilometers.

    This is the distance "as the crow flies" over the Earth's curved
    surface -- not a road distance. We use it in two places:
      1) as the distance-matrix fallback when OSMnx / internet isn't
         available (scaled by a "detour factor", see distance_matrix.py),
      2) as the edge cost inside the local A* grid search (nofly_astar.py),
         since a drone flying around a no-fly zone travels in genuinely
         straight segments through open air, not along roads.
    """
    lat1, lon1 = p1
    lat2, lon2 = p2
    R = 6371.0088  # mean Earth radius in km

    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def point_in_polygon(point: Point, polygon: Polygon) -> bool:
    """Classic ray-casting point-in-polygon test.

    Idea: cast an imaginary horizontal ray from `point` out to +infinity
    longitude. Count how many polygon edges that ray crosses. Odd number of
    crossings => point is inside; even => outside. (This is the same trick
    used to decide if a dot is inside a hand-drawn shape by counting how many
    times a line from the dot to the edge of the page crosses the outline.)

    `polygon` is a list of (lat, lon) vertices; treated as a closed ring
    (we wrap from the last vertex back to the first).
    """
    lat, lon = point
    n = len(polygon)
    inside = False

    j = n - 1  # previous vertex index, wrapping around
    for i in range(n):
        lat_i, lon_i = polygon[i]
        lat_j, lon_j = polygon[j]

        # Does the edge (i, j) straddle this point's latitude?
        straddles = (lat_i > lat) != (lat_j > lat)
        if straddles:
            # Longitude where the edge crosses this point's latitude line
            lon_intersect = lon_i + (lat - lat_i) * (lon_j - lon_i) / (lat_j - lat_i)
            if lon < lon_intersect:
                inside = not inside
        j = i

    return inside


def _orientation(a: Point, b: Point, c: Point) -> int:
    """Sign of the cross product (b-a) x (c-a). Used by segment intersection.
    >0 => counter-clockwise turn, <0 => clockwise, 0 => collinear."""
    val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
    if val > 1e-12:
        return 1
    elif val < -1e-12:
        return -1
    return 0


def _on_segment(a: Point, b: Point, c: Point) -> bool:
    """True if collinear point b lies on segment a-c."""
    return (min(a[0], c[0]) <= b[0] <= max(a[0], c[0])
            and min(a[1], c[1]) <= b[1] <= max(a[1], c[1]))


def _segments_intersect(p1: Point, p2: Point, p3: Point, p4: Point) -> bool:
    """Standard O(1) segment-segment intersection test using orientations."""
    o1 = _orientation(p1, p2, p3)
    o2 = _orientation(p1, p2, p4)
    o3 = _orientation(p3, p4, p1)
    o4 = _orientation(p3, p4, p2)

    if o1 != o2 and o3 != o4:
        return True

    # Collinear special cases
    if o1 == 0 and _on_segment(p1, p3, p2):
        return True
    if o2 == 0 and _on_segment(p1, p4, p2):
        return True
    if o3 == 0 and _on_segment(p3, p1, p4):
        return True
    if o4 == 0 and _on_segment(p3, p2, p4):
        return True
    return False


def point_to_segment_distance_km(p: Point, a: Point, b: Point) -> float:
    """Distance from point p to segment a-b in km. Approximated via equirectangular projection."""
    lat_p, lon_p = p
    lat_a, lon_a = a
    lat_b, lon_b = b
    
    # Scale longitude to roughly match latitude scale at the middle latitude
    mid_lat = math.radians((lat_p + lat_a + lat_b) / 3.0)
    cos_lat = math.cos(mid_lat)
    
    px, py = lon_p * cos_lat, lat_p
    ax, ay = lon_a * cos_lat, lat_a
    bx, by = lon_b * cos_lat, lat_b
    
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return haversine_km(p, a)
    
    t = ((px - ax) * dx + (py - ay) * dy) / (dx*dx + dy*dy)
    t = max(0.0, min(1.0, t))
    
    cx, cy = ax + t * dx, ay + t * dy
    # Avoid div by zero if cos_lat is very small, though we are not at poles.
    c_lat, c_lon = cy, cx / max(cos_lat, 0.00001)
    
    return haversine_km(p, (c_lat, c_lon))

def point_near_polygon(point: Point, polygon: Polygon, margin_m: float) -> bool:
    """True if point is inside polygon or within margin_m of any edge."""
    margin_km = margin_m / 1000.0 if margin_m > 0 else 0.0
    pad_lat = margin_km / 111.0
    p_lat, p_lon = point

    min_lat = min(v[0] for v in polygon)
    max_lat = max(v[0] for v in polygon)
    if p_lat < min_lat - pad_lat or p_lat > max_lat + pad_lat:
        return False

    max_lat_mag = max(abs(min_lat), abs(max_lat))
    cos_lat = max(math.cos(math.radians(max_lat_mag)), 0.1)
    pad_lon = margin_km / (111.0 * cos_lat)
    min_lon = min(v[1] for v in polygon)
    max_lon = max(v[1] for v in polygon)
    if p_lon < min_lon - pad_lon or p_lon > max_lon + pad_lon:
        return False

    if point_in_polygon(point, polygon):
        return True
    if margin_m <= 0:
        return False
        
    n = len(polygon)
    for i in range(n):
        if point_to_segment_distance_km(point, polygon[i], polygon[(i+1)%n]) <= margin_km:
            return True
    return False

def segment_intersects_polygon(p1: Point, p2: Point, polygon: Polygon, margin_m: float = 0.0) -> bool:
    """Does the straight segment p1->p2 pass through or come within margin_m of `polygon`?"""
    margin_km = margin_m / 1000.0 if margin_m > 0 else 0.0
    pad_lat = margin_km / 111.0
    min_poly_lat = min(v[0] for v in polygon)
    max_poly_lat = max(v[0] for v in polygon)

    min_seg_lat = min(p1[0], p2[0])
    max_seg_lat = max(p1[0], p2[0])
    if max_seg_lat < min_poly_lat - pad_lat or min_seg_lat > max_poly_lat + pad_lat:
        return False

    max_lat_mag = max(abs(min_poly_lat), abs(max_poly_lat))
    cos_lat = max(math.cos(math.radians(max_lat_mag)), 0.1)
    pad_lon = margin_km / (111.0 * cos_lat)
    min_poly_lon = min(v[1] for v in polygon)
    max_poly_lon = max(v[1] for v in polygon)

    min_seg_lon = min(p1[1], p2[1])
    max_seg_lon = max(p1[1], p2[1])
    if max_seg_lon < min_poly_lon - pad_lon or min_seg_lon > max_poly_lon + pad_lon:
        return False

    if point_near_polygon(p1, polygon, margin_m) or point_near_polygon(p2, polygon, margin_m):
        return True

    n = len(polygon)
    for i in range(n):
        edge_a = polygon[i]
        edge_b = polygon[(i + 1) % n]
        if _segments_intersect(p1, p2, edge_a, edge_b):
            return True
            
        if margin_m > 0:
            # Also check if any polygon vertex is dangerously close to the flight path
            if point_to_segment_distance_km(edge_a, p1, p2) <= margin_km:
                return True
                
    return False



def path_intersects_any_zone(p1: Point, p2: Point, zones: List[Polygon], margin_m: float = 0.0) -> bool:
    """Convenience: does segment p1->p2 cross ANY of the given no-fly zones?"""
    return any(segment_intersects_polygon(p1, p2, zone, margin_m) for zone in zones)


def polygon_bounds(polygon: Polygon, margin_km: float = 0.0) -> Tuple[float, float, float, float]:
    """Bounding box (min_lat, min_lon, max_lat, max_lon) of a polygon, padded
    by `margin_km` on every side. Used to size the local A* search grid."""
    lats = [p[0] for p in polygon]
    lons = [p[1] for p in polygon]
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons), max(lons)

    if margin_km > 0:
        # Rough conversion: 1 degree latitude ~= 111 km everywhere;
        # 1 degree longitude ~= 111 km * cos(latitude). Good enough for
        # padding a search grid by a couple hundred meters to a few km.
        mid_lat_rad = math.radians((min_lat + max_lat) / 2)
        dlat = margin_km / 111.0
        dlon = margin_km / (111.0 * max(math.cos(mid_lat_rad), 0.1))
        min_lat, max_lat = min_lat - dlat, max_lat + dlat
        min_lon, max_lon = min_lon - dlon, max_lon + dlon

    return min_lat, min_lon, max_lat, max_lon
