"""
load_scenario.py
================
Universal scenario loader for custom configurations.

Parses a JSON file describing the depot, deliveries, no-fly zones, and
drone specs into a CityConfig object that can be passed to the solver.

EXAMPLE JSON (save as my_city.json):
{
    "name": "New York Hurricane Response",
    "scenario_description": "Post-hurricane medical delivery in Lower Manhattan",
    "depot": {
        "name": "NYU Langone Hospital",
        "lat": 40.7421,
        "lon": -73.9739
    },
    "drone": {
        "capacity_kg": 8.0,
        "max_range_km": 35.0,
        "cruise_speed_kmh": 45.0
    },
    "deliveries": [
        {
            "name": "Battery Park Shelter",
            "lat": 40.7033,
            "lon": -74.0170,
            "demand_kg": 3.5,
            "urgency": "critical",
            "window_minutes": 20,
            "cold_chain_limit_minutes": 30
        },
        {
            "name": "Chinatown Relief Camp",
            "lat": 40.7158,
            "lon": -73.9970,
            "demand_kg": 5.0,
            "urgency": "urgent",
            "window_minutes": 40
        }
    ],
    "no_fly_zones": [
        [
            [40.710, -74.015],
            [40.710, -74.005],
            [40.715, -74.005],
            [40.715, -74.015]
        ]
    ]
}

USAGE:
    python3 main.py --from-json my_city.json
    python3 simulate_fleet.py --from-json my_city.json --events 3
    streamlit run dashboard.py  (then upload JSON in the sidebar)

This means ANYONE can use this system for ANY city on Earth by writing
a simple JSON file.  No Python knowledge required.  No code changes.

GPS / NAVIC / GLONASS NOTE:
    This system works with WGS84 latitude/longitude coordinates — the same
    coordinate system used by GPS (US), NAVIC/IRNSS (India), GLONASS (Russia),
    Galileo (EU), and BeiDou (China).  All these satellite navigation systems
    output WGS84 lat/lon, so this system is inherently compatible with ALL of
    them.  You don't "integrate GPS" — GPS is just the device that gives you
    the lat/lon numbers.  Those numbers go into this JSON file.
"""

import json
import os
from typing import Optional

from config import Location, DroneConfig, CityConfig
from geo_utils import point_near_polygon


def load_scenario_from_json(filepath: str) -> CityConfig:
    """Parse a JSON scenario file into a CityConfig.

    This is the function that makes the system work ANYWHERE IN THE WORLD.
    The user writes a JSON file with GPS coordinates for their specific
    location, and this function converts it into the exact same CityConfig
    object that the hardcoded Jaipur/Chennai scenarios use.

    The routing engine, simulator, dashboard — everything — works identically
    because they only see a CityConfig.  They don't care where it came from.
    """
    with open(filepath, "r") as f:
        data = json.load(f)

    # Parse depot
    d = data["depot"]
    depot = Location(
        id=0,
        name=d["name"],
        lat=d["lat"],
        lon=d["lon"],
        demand_kg=0.0,
    )

    # Parse deliveries
    deliveries = []
    for i, stop in enumerate(data["deliveries"]):
        deliveries.append(Location(
            id=i + 1,
            name=stop["name"],
            lat=stop["lat"],
            lon=stop["lon"],
            demand_kg=stop.get("demand_kg", 1.0),
            urgency=stop.get("urgency", "routine"),
            window_minutes=stop.get("window_minutes"),
            cold_chain_limit_minutes=stop.get("cold_chain_limit_minutes"),
            landing_zone_image=stop.get("landing_zone_image"),
        ))

    # Parse drone config
    drone_data = data.get("drone", {})
    drone = DroneConfig(
        capacity_kg=drone_data.get("capacity_kg", 10.0),
        max_range_km=drone_data.get("max_range_km", 40.0),
        cruise_speed_kmh=drone_data.get("cruise_speed_kmh", 40.0),
        fleet_size=drone_data.get("fleet_size", 5),
        turnaround_min=drone_data.get("turnaround_min", 10.0),
        dwell_min=drone_data.get("dwell_min", 2.0),
        reserve_fraction=drone_data.get("reserve_fraction", 0.10),
        safety_margin_m=drone_data.get("safety_margin_m", 50.0),
    )

    # Parse no-fly zones
    no_fly_zones = []
    for zone in data.get("no_fly_zones", []):
        no_fly_zones.append([(pt[0], pt[1]) for pt in zone])

    city = CityConfig(
        name=data.get("name", os.path.basename(filepath)),
        depot=depot,
        deliveries=deliveries,
        no_fly_zones=no_fly_zones,
        drone=drone,
        scenario_description=data.get("scenario_description", ""),
    )
    
    validate_scenario(city)
    return city

def validate_scenario(city: CityConfig):
    """Validates that all locations, drone configurations, and no-fly zones
    meet structural, physical, and safety constraints.
    
    Checks:
      - Valid latitude (-90 to 90) and longitude (-180 to 180) for all locations.
      - Non-negative demand and timing values.
      - Positive drone capacity, range, and speed.
      - Fleet size >= 1 and 0.0 <= reserve_fraction < 1.0.
      - Non-negative turnaround, dwell, and safety margins.
      - Valid NFZ polygons (>= 3 vertices, coordinates in range).
      - Locations are outside NFZs plus drone safety_margin_m buffer.
    """
    if not (-90.0 <= city.depot.lat <= 90.0 and -180.0 <= city.depot.lon <= 180.0):
        raise ValueError(f"Depot latitude/longitude out of range: ({city.depot.lat}, {city.depot.lon})")

    drone = city.drone
    if drone is not None:
        if drone.capacity_kg <= 0:
            raise ValueError(f"Drone capacity_kg must be positive, got {drone.capacity_kg}")
        if drone.max_range_km <= 0:
            raise ValueError(f"Drone max_range_km must be positive, got {drone.max_range_km}")
        if drone.cruise_speed_kmh <= 0:
            raise ValueError(f"Drone cruise_speed_kmh must be positive, got {drone.cruise_speed_kmh}")
        if drone.fleet_size < 1:
            raise ValueError(f"Drone fleet_size must be >= 1, got {drone.fleet_size}")
        if not (0.0 <= drone.reserve_fraction < 1.0):
            raise ValueError(f"Drone reserve_fraction must be in [0.0, 1.0), got {drone.reserve_fraction}")
        if drone.turnaround_min < 0:
            raise ValueError(f"Drone turnaround_min must be >= 0, got {drone.turnaround_min}")
        if drone.dwell_min < 0:
            raise ValueError(f"Drone dwell_min must be >= 0, got {drone.dwell_min}")
        if drone.safety_margin_m < 0:
            raise ValueError(f"Drone safety_margin_m must be >= 0, got {drone.safety_margin_m}")

    safety_margin_m = drone.safety_margin_m if drone else 50.0

    for idx, zone in enumerate(city.no_fly_zones):
        if len(zone) < 3:
            raise ValueError(f"No-fly zone {idx} has fewer than 3 vertices ({len(zone)})")
        for pt in zone:
            if not (-90.0 <= pt[0] <= 90.0 and -180.0 <= pt[1] <= 180.0):
                raise ValueError(f"No-fly zone {idx} vertex ({pt[0]}, {pt[1]}) coordinates out of range")

    for zone in city.no_fly_zones:
        if point_near_polygon((city.depot.lat, city.depot.lon), zone, safety_margin_m):
            raise ValueError(f"Depot {city.depot.name} is inside or within {safety_margin_m}m of a no-fly zone!")

    for d in city.deliveries:
        if not (-90.0 <= d.lat <= 90.0 and -180.0 <= d.lon <= 180.0):
            raise ValueError(f"Delivery {d.name} coordinates out of range: ({d.lat}, {d.lon})")
        if d.demand_kg < 0:
            raise ValueError(f"Delivery {d.name} demand_kg cannot be negative: {d.demand_kg}")
        if d.window_minutes is not None and d.window_minutes < 0:
            raise ValueError(f"Delivery {d.name} window_minutes cannot be negative: {d.window_minutes}")
        if d.cold_chain_limit_minutes is not None and d.cold_chain_limit_minutes < 0:
            raise ValueError(f"Delivery {d.name} cold_chain_limit_minutes cannot be negative: {d.cold_chain_limit_minutes}")
        if getattr(d, 'request_time_min', 0.0) < 0:
            raise ValueError(f"Delivery {d.name} request_time_min cannot be negative")

        for zone in city.no_fly_zones:
            if point_near_polygon((d.lat, d.lon), zone, safety_margin_m):
                raise ValueError(f"Delivery {d.name} is inside or within {safety_margin_m}m of a no-fly zone!")


def generate_random_scenario(
    center_lat: float,
    center_lon: float,
    num_deliveries: int = 8,
    num_no_fly_zones: int = 2,
    radius_km: float = 15.0,
    name: str = "Random Scenario",
    seed: Optional[int] = None,
) -> CityConfig:
    """Generate a structurally valid random scenario centered on any GPS point with bounded retries.
    
    Generates well-formed scenario geometry: valid coordinates, non-negative demands,
    and depot/deliveries strictly outside no-fly zones including safety buffers.
    Note that randomly generated urgency, cold-chain limits, and spatial configurations
    are stochastic test cases and may not always be solvable under hard physical constraints.
    """
    import random
    rng = random.Random(seed)
    spread = radius_km / 111.0  # rough degrees per km
    max_attempts = 100

    for attempt in range(max_attempts):
        # Generate random no-fly zones
        no_fly_zones = []
        for _ in range(num_no_fly_zones):
            clat = center_lat + rng.uniform(-spread * 0.7, spread * 0.7)
            clon = center_lon + rng.uniform(-spread * 0.7, spread * 0.7)
            size = rng.uniform(0.005, 0.015)
            no_fly_zones.append([
                (clat - size, clon - size),
                (clat - size, clon + size),
                (clat + size, clon + size),
                (clat + size, clon - size),
            ])

        drone = DroneConfig(
            capacity_kg=10.0,
            max_range_km=max(40.0, radius_km * 3),
            cruise_speed_kmh=40.0,
        )

        depot = Location(
            id=0,
            name=f"Central Depot ({center_lat:.4f}, {center_lon:.4f})",
            lat=center_lat,
            lon=center_lon,
            demand_kg=0.0,
        )
        
        # Check depot safety
        depot_blocked = False
        for zone in no_fly_zones:
            if point_near_polygon((depot.lat, depot.lon), zone, drone.safety_margin_m):
                depot_blocked = True
                break
        if depot_blocked:
            continue

        deliveries = []
        urgency_choices = ["critical", "urgent", "routine"]
        urgency_weights = [0.2, 0.3, 0.5]

        valid_deliveries = True
        for i in range(num_deliveries):
            dlat = center_lat + rng.uniform(-spread, spread)
            dlon = center_lon + rng.uniform(-spread, spread)
            
            # Check NFZ distance
            in_nfz = False
            for zone in no_fly_zones:
                if point_near_polygon((dlat, dlon), zone, drone.safety_margin_m):
                    in_nfz = True
                    break
            if in_nfz:
                valid_deliveries = False
                break
                        
            urgency = rng.choices(urgency_choices, urgency_weights, k=1)[0]
            demand = round(rng.uniform(1.0, 8.0), 1)

            window = None
            if urgency == "critical":
                window = rng.choice([20, 25, 30])
            elif urgency == "urgent":
                window = rng.choice([35, 40, 45])

            cold_chain = None
            if rng.random() < 0.3:
                cold_chain = rng.choice([30, 45, 60])

            deliveries.append(Location(
                id=i + 1,
                name=f"Delivery Point {i + 1} ({dlat:.4f}, {dlon:.4f})",
                lat=dlat,
                lon=dlon,
                demand_kg=demand,
                urgency=urgency,
                window_minutes=window,
                cold_chain_limit_minutes=cold_chain,
            ))

        if not valid_deliveries:
            continue

        city = CityConfig(
            name=name,
            depot=depot,
            deliveries=deliveries,
            no_fly_zones=no_fly_zones,
            drone=drone,
            scenario_description=f"Randomly generated scenario with {num_deliveries} "
                                 f"deliveries within {radius_km}km of "
                                 f"({center_lat:.4f}, {center_lon:.4f})",
        )

        try:
            validate_scenario(city)
            return city
        except ValueError:
            continue

    raise ValueError(f"Could not generate a structurally valid random scenario after {max_attempts} attempts.")


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 load_scenario.py my_city.json     # Load from JSON file")
        print("  python3 load_scenario.py --random LAT LON # Random scenario at GPS point")
        print()
        print("Examples:")
        print("  python3 load_scenario.py scenario.json")
        print("  python3 load_scenario.py --random 40.7580 -73.9855    # New York")
        print("  python3 load_scenario.py --random 35.6586 139.7454    # Tokyo")
        print("  python3 load_scenario.py --random 28.6139 77.2090     # New Delhi")
        sys.exit(0)

    if sys.argv[1] == "--random":
        lat = float(sys.argv[2])
        lon = float(sys.argv[3])
        city = generate_random_scenario(lat, lon)
        print(f"Generated random scenario at ({lat}, {lon})")
        print(f"  Depot: {city.depot.name}")
        print(f"  Deliveries: {len(city.deliveries)}")
        print(f"  No-fly zones: {len(city.no_fly_zones)}")

        # Verify it works
        from distance_matrix import build_flight_distance_matrix
        from vrp_scratch import solve_vrp_from_scratch, total_distance

        matrix, detours = build_flight_distance_matrix(
            city.all_locations, city.no_fly_zones, drone_config=city.drone
        )
        routes = solve_vrp_from_scratch(city.all_locations, matrix, city.drone)
        print(f"  Routes: {len(routes)}, Total distance: {total_distance(routes, matrix):.1f} km")
        print("  ✅ Works anywhere in the world.")
    else:
        city = load_scenario_from_json(sys.argv[1])
        print(f"Loaded scenario: {city.name}")
        print(f"  Depot: {city.depot.name} ({city.depot.lat}, {city.depot.lon})")
        print(f"  Deliveries: {len(city.deliveries)}")
        print(f"  No-fly zones: {len(city.no_fly_zones)}")
        print(f"  Drone: {city.drone.capacity_kg}kg, {city.drone.max_range_km}km range")
