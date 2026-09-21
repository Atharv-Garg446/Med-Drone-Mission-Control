"""
visualize_map.py
=================
Renders an animated folium map: a marker that moves along each solved
route over time, with no-fly zones drawn as red polygons, the route
visibly bending around them where a detour was needed, and a popup at
every animation frame showing live range-remaining and cargo-remaining.

REQUIRES: `pip install folium`. Uses folium.plugins.TimestampedGeoJson,
which renders a play/pause/scrub timeline control in the browser and steps
a GeoJSON feature's position frame-by-frame -- this is a real folium
plugin (not custom JS), documented at
https://python-visualization.github.io/folium/latest/user_guide/plugins/timestamped_geojson.html

HOW THE ANIMATION IS BUILT
---------------------------
1. For every leg of a route (stop A -> stop B), we already know its real
   flight path from distance_matrix.py's `detours` dict: either a straight
   [A, B] line, or the bent A*-computed waypoint list if that leg had to
   dodge a no-fly zone.
2. We walk along that path at a fixed simulated time step (default: every
   30 simulated seconds), interpolating intermediate lat/lon points so the
   marker moves smoothly rather than teleporting stop-to-stop.
3. At each simulated instant we also know the cumulative distance flown so
   far and the cumulative demand delivered so far, so "range remaining"
   and "cargo remaining" fall out as simple arithmetic against the drone's
   max_range_km / capacity_kg.
"""

import datetime
from typing import List, Dict, Tuple
from config import Location, DroneConfig
from geo_utils import haversine_km

ROUTE_COLORS = ["#2166AC", "#B2182B", "#1A9850", "#762A83", "#E08214", "#4393C3"]


def _leg_path(i: int, j: int, locations: List[Location], detours: Dict[Tuple[int, int], list]):
    """The real flight-path geometry for one leg of a route: either the
    straight [A, B] line, or the A*-detour waypoint list if this exact
    (i, j) pair needed to route around a no-fly zone."""
    if (i, j) in detours:
        return detours[(i, j)]
    return [(locations[i].lat, locations[i].lon), (locations[j].lat, locations[j].lon)]


def _interpolate_along_path(path: List[Tuple[float, float]], step_km: float):
    """Walk a multi-point path and yield (lat, lon, cumulative_km_along_this_leg)
    every `step_km`, plus the final endpoint exactly."""
    cumulative = 0.0
    yield path[0][0], path[0][1], 0.0

    for a, b in zip(path[:-1], path[1:]):
        seg_len = haversine_km(a, b)
        if seg_len < 1e-9:
            continue
        traveled = 0.0
        while traveled + step_km < seg_len:
            traveled += step_km
            frac = traveled / seg_len
            lat = a[0] + (b[0] - a[0]) * frac
            lon = a[1] + (b[1] - a[1]) * frac
            cumulative += step_km
            yield lat, lon, cumulative
        cumulative += (seg_len - traveled)
        yield b[0], b[1], cumulative


def build_route_animation_features(
    route: List[int],
    route_index: int,
    locations: List[Location],
    drone: DroneConfig,
    detours: Dict[Tuple[int, int], list],
    start_time: datetime.datetime,
    step_km: float = 0.5,
) -> list:
    """Build the list of GeoJSON Point features (one per animation frame)
    for a single route, each carrying a timestamp and live range/cargo
    telemetry in its `popup`/`tooltip` properties -- this is the format
    folium.plugins.TimestampedGeoJson expects."""
    features = []
    cumulative_km = 0.0
    remaining_capacity = sum(locations[node].demand_kg for node in route[1:-1])
    usable_range = drone.max_range_km * (1.0 - drone.reserve_fraction)
    seconds_per_km = 3600.0 / drone.cruise_speed_kmh

    for step in range(len(route) - 1):
        i, j = route[step], route[step + 1]
        path = _leg_path(i, j, locations, detours)

        last_leg_km = 0.0
        for lat, lon, leg_km in _interpolate_along_path(path, step_km):
            last_leg_km = leg_km
            total_km_here = cumulative_km + leg_km
            range_pct = max(0.0, 100.0 * (1.0 - total_km_here / usable_range)) if usable_range > 0 else 0.0
            timestamp = start_time + datetime.timedelta(seconds=total_km_here * seconds_per_km)

            features.append({
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [lon, lat]},
                "properties": {
                    "time": timestamp.isoformat(),
                    "icon": "circle",
                    "iconstyle": {
                        "fillColor": ROUTE_COLORS[route_index % len(ROUTE_COLORS)],
                        "fillOpacity": 0.9,
                        "stroke": "true",
                        "radius": 7,
                    },
                    "popup": (f"Route {route_index + 1}<br>"
                              f"Range: {range_pct:.0f}%<br>"
                              f"Cargo remaining: {max(0.0, remaining_capacity):.1f} kg<br>"
                              f"Distance flown: {total_km_here:.2f} km"),
                },
            })

        # `leg_km` from the last interpolated point already equals this
        # leg's full path length (straight or detoured) -- fold it into
        # the running total rather than re-summing the path a second time.
        cumulative_km += last_leg_km

        # Cargo drops off the instant we arrive at node j (unless j is the
        # depot, which has zero demand anyway so this is a no-op there).
        remaining_capacity -= locations[j].demand_kg

    return features


def render_map(
    routes: List[List[int]],
    locations: List[Location],
    drone: DroneConfig,
    no_fly_zones,
    detours: Dict[Tuple[int, int], list],
    output_path: str = "output/drone_routes_animated.html",
):
    """Build and save the full animated folium map. Returns the output path."""
    try:
        import folium
        from folium.plugins import TimestampedGeoJson
    except ImportError:
        print("[info] folium not installed -- skipping map visualization "
              "(`pip install folium` to enable it). Every other output "
              "(routes, waypoint files, benchmark numbers) is unaffected.")
        return None

    depot = locations[0]
    fmap = folium.Map(location=[depot.lat, depot.lon], zoom_start=12, tiles="OpenStreetMap")

    # --- draw no-fly zones first, so routes/markers layer on top --------
    for zone in no_fly_zones:
        folium.Polygon(
            locations=[(lat, lon) for lat, lon in zone],
            color="#B2182B", weight=2, fill=True, fill_color="#B2182B", fill_opacity=0.25,
            popup="No-fly zone",
        ).add_to(fmap)

    # --- static markers: depot + every delivery stop ---------------------
    folium.Marker(
        [depot.lat, depot.lon],
        popup=f"DEPOT: {depot.name}",
        icon=folium.Icon(color="black", icon="home"),
    ).add_to(fmap)

    for loc in locations[1:]:
        folium.Marker(
            [loc.lat, loc.lon],
            popup=f"{loc.name}<br>Demand: {loc.demand_kg}kg<br>Urgency: {loc.urgency}",
            icon=folium.Icon(color="blue", icon="plus", prefix="fa"),
        ).add_to(fmap)

    # --- static polylines for each route, so the shape is visible even
    #     before/after the animation plays, and shows the no-fly detour bend
    for r_idx, route in enumerate(routes):
        full_path = []
        for step in range(len(route) - 1):
            i, j = route[step], route[step + 1]
            leg = _leg_path(i, j, locations, detours)
            full_path.extend(leg if not full_path else leg[1:])
        folium.PolyLine(
            full_path, color=ROUTE_COLORS[r_idx % len(ROUTE_COLORS)], weight=3, opacity=0.7,
            tooltip=f"Route {r_idx + 1}",
        ).add_to(fmap)

    # --- animated marker layer --------------------------------------------
    start_time = datetime.datetime(2026, 1, 1, 8, 0, 0)  # arbitrary fixed mission-start clock time
    all_features = []
    for r_idx, route in enumerate(routes):
        all_features.extend(
            build_route_animation_features(route, r_idx, locations, drone, detours, start_time)
        )

    TimestampedGeoJson(
        {"type": "FeatureCollection", "features": all_features},
        period="PT30S",          # one animation frame per 30 simulated seconds
        add_last_point=False,
        auto_play=True,
        loop=False,
        max_speed=10,
        loop_button=True,
        date_options="HH:mm:ss",
        time_slider_drag_update=True,
    ).add_to(fmap)

    import os
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    fmap.save(output_path)
    return output_path
