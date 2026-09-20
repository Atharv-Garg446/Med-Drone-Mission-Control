"""
visualize_simulation.py
=======================
Generates an animated HTML map that VISUALLY REPLAYS a fleet simulation.

You will SEE:
  - Drone icons moving along their routes on a real map (OpenStreetMap)
  - No-fly zones drawn as red polygons (static ones + dynamic TFRs appearing)
  - Emergency delivery points popping up as red markers
  - Delivery completion markers turning green
  - The depot marked as a black home icon
  - All delivery stops as blue medical markers

The animation uses folium's TimestampedGeoJson plugin, which gives you a
play/pause/scrub timeline in the browser.

REQUIRES: pip install folium

Usage (called automatically by simulate_fleet.py --visualize):
    sim = FleetSimulator(city, events=events)
    report = sim.run()
    render_simulation_map(sim, "output/simulation_replay.html")
    # Then open output/simulation_replay.html in your browser
"""

import datetime
import os
from typing import List, Dict

DRONE_COLORS = ["#2166AC", "#B2182B", "#1A9850", "#762A83", "#E08214",
                "#4393C3", "#D6604D", "#66C2A5", "#FC8D62", "#8DA0CB"]


def render_simulation_map(sim, output_path: str = "output/simulation_replay.html") -> str:
    """Build an animated folium map from a completed FleetSimulator.

    Returns the output path, or None if folium is not installed.
    """
    try:
        import folium
        from folium.plugins import TimestampedGeoJson
    except ImportError:
        print("[info] folium not installed — skipping visual simulation map "
              "(`pip install folium` to enable). The terminal log is still fully functional.")
        return None

    city = sim.city
    locations = sim.locations
    depot = locations[0]

    # Center the map on the depot
    fmap = folium.Map(location=[depot.lat, depot.lon], zoom_start=12,
                      tiles="OpenStreetMap")

    # --- Layer 1: Static no-fly zones (existed before the simulation) ---
    for zone in city.no_fly_zones:
        folium.Polygon(
            locations=[(lat, lon) for lat, lon in zone],
            color="#B2182B", weight=2, fill=True, fill_color="#B2182B",
            fill_opacity=0.25, popup="Original No-Fly Zone",
        ).add_to(fmap)

    # --- Layer 2: Dynamic TFRs (appeared during the simulation) ---
    for nfz in sim.nfz_history:
        zone = nfz["zone"]
        reason = nfz.get("reason", "Pop-up TFR")
        tick = nfz["tick"]
        minutes = tick // 60
        seconds = tick % 60
        folium.Polygon(
            locations=[(lat, lon) for lat, lon in zone],
            color="#FF6600", weight=3, fill=True, fill_color="#FF6600",
            fill_opacity=0.35,
            popup=f"⚡ {reason}<br>Appeared at T+{minutes:02d}:{seconds:02d}",
            tooltip=f"TFR @ T+{minutes}:{seconds:02d}",
        ).add_to(fmap)

    # --- Layer 3: Depot marker ---
    folium.Marker(
        [depot.lat, depot.lon],
        popup=f"<b>DEPOT:</b> {depot.name}",
        icon=folium.Icon(color="black", icon="home", prefix="fa"),
    ).add_to(fmap)

    # --- Layer 4: Delivery stop markers ---
    for loc in locations[1:]:
        folium.Marker(
            [loc.lat, loc.lon],
            popup=f"<b>{loc.name}</b><br>Demand: {loc.demand_kg}kg<br>Urgency: {loc.urgency}",
            icon=folium.Icon(color="blue", icon="plus-sign"),
        ).add_to(fmap)

    # --- Layer 5: Emergency delivery markers (appeared mid-simulation) ---
    for event in sim.event_history:
        if event["type"] == "emergency":
            tick = event["tick"]
            m, s = tick // 60, tick % 60
            folium.Marker(
                [event["lat"], event["lon"]],
                popup=f"<b>🚨 EMERGENCY</b><br>{event.get('name', 'Emergency')}<br>"
                      f"Appeared at T+{m:02d}:{s:02d}",
                icon=folium.Icon(color="red", icon="exclamation-sign"),
            ).add_to(fmap)

    # --- Layer 6: Delivery completion markers ---
    for delivery in sim.delivery_history:
        tick = delivery["tick"]
        m, s = tick // 60, tick % 60
        folium.CircleMarker(
            [delivery["lat"], delivery["lon"]],
            radius=8, color="#1A9850", fill=True, fill_color="#1A9850",
            fill_opacity=0.8,
            popup=f"✅ Delivered by Drone {delivery['drone_id']}<br>"
                  f"{delivery['name']}<br>T+{m:02d}:{s:02d}",
        ).add_to(fmap)

    # --- Layer 7: Animated drone movement (the main visual) ---
    if sim.position_history:
        # Group positions by drone_id
        by_drone: Dict[int, list] = {}
        for pos in sim.position_history:
            by_drone.setdefault(pos["drone_id"], []).append(pos)

        # Build TimestampedGeoJson features for each drone
        # Use an arbitrary start datetime for the animation timeline
        base_time = datetime.datetime(2026, 1, 1, 8, 0, 0)
        all_features = []

        for drone_id, positions in by_drone.items():
            color = DRONE_COLORS[(drone_id - 1) % len(DRONE_COLORS)]

            for pos in positions:
                t = base_time + datetime.timedelta(seconds=pos["tick"])
                all_features.append({
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [pos["lon"], pos["lat"]],
                    },
                    "properties": {
                        "time": t.isoformat(),
                        "icon": "circle",
                        "iconstyle": {
                            "fillColor": color,
                            "fillOpacity": 0.9,
                            "stroke": "true",
                            "color": "#ffffff",
                            "weight": 2,
                            "radius": 8,
                        },
                        "popup": f"Drone {drone_id}",
                        "tooltip": f"Drone {drone_id}",
                    },
                })

        if all_features:
            TimestampedGeoJson(
                {"type": "FeatureCollection", "features": all_features},
                period="PT5S",  # one frame per 5 simulated seconds
                add_last_point=False,
                auto_play=True,
                loop=False,
                max_speed=10,
                loop_button=True,
                date_options="HH:mm:ss",
                time_slider_drag_update=True,
            ).add_to(fmap)

        # Also draw static polylines showing the paths each drone took
        for drone_id, positions in by_drone.items():
            if len(positions) < 2:
                continue
            color = DRONE_COLORS[(drone_id - 1) % len(DRONE_COLORS)]
            path = [(p["lat"], p["lon"]) for p in positions]
            folium.PolyLine(
                path, color=color, weight=2, opacity=0.5,
                tooltip=f"Drone {drone_id} path",
                dash_array="5",
            ).add_to(fmap)

    # Save
    os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else ".", exist_ok=True)
    fmap.save(output_path)
    return output_path
