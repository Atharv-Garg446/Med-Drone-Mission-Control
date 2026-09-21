"""
main.py
=======
Runs the drone route optimization pipeline end-to-end.

Computes optimal drone routes from a central hospital depot to relief camps 
and emergency medical posts, respecting real physical constraints (payload, 
range, no-fly zones, cold chain).

Pipeline steps:
  1. Build the flight-distance matrix (no-fly-zone aware, with A* detours)
  2. Solve the CVRP+range+cold-chain problem with three from-scratch heuristics
  3. Benchmark against Google OR-Tools and print the % gap
  4. Check delivery-window urgency against the best solution
  5. Check cold-chain compliance (cumulative refrigeration time)
  6. Run the YOLOv8 landing-zone safety check on any configured images
  7. Render the animated folium map
  8. Export every route as a QGC WPL 110 .waypoints file (with A* detours)
  9. Generate static matplotlib plots

Every optional step degrades gracefully if its dependency isn't installed.

Usage:
    python3 main.py                          # Jaipur disaster (default)
    python3 main.py --city jaipur_hospital   # Jaipur hospital network
    python3 main.py --city chennai_flood     # Chennai cyclone flood
    python3 main.py --no-osmnx              # skip OSMnx road network
    python3 main.py --experiments           # also run the full experiment suite
"""

import argparse
import sys

from config import CityConfig, JAIPUR_DISASTER, CITY_CONFIGS
from distance_matrix import build_flight_distance_matrix, build_osmnx_reference_matrix
from vrp_scratch import (
    solve_vrp_from_scratch, solve_vrp_with_heuristic,
    total_distance, _CONSTRUCTION_HEURISTICS,
)
from vrp_ortools import solve_vrp_with_ortools
from time_windows import check_time_windows
from cold_chain import check_cold_chain_all_routes
from yolo_safety import check_landing_zones
from visualize_map import render_map
from waypoint_export import export_all_routes


def print_routes(routes, locations, matrix, label):
    print(f"\n--- {label} ---")
    for i, route in enumerate(routes):
        names = " -> ".join(locations[n].name.split(",")[0] for n in route)
        dist = sum(matrix[route[k]][route[k + 1]] for k in range(len(route) - 1))
        demand = sum(locations[n].demand_kg for n in route[1:-1])
        print(f"  Route {i + 1}: {names}")
        print(f"    distance={dist:.2f} km | payload={demand:.1f}kg | stops={len(route) - 2}")
    print(f"  TOTAL distance: {total_distance(routes, matrix):.2f} km")


def run(city: CityConfig, use_osmnx: bool = True, run_experiments: bool = False):
    locations = city.all_locations
    print(f"=== Drone Medical-Supply Route Optimizer: {city.name} ===")
    if city.scenario_description:
        print(f"\nScenario: {city.scenario_description}")
    print(f"\n{len(locations) - 1} delivery locations, drone capacity "
          f"{city.drone.capacity_kg}kg, max range {city.drone.max_range_km}km")

    # --- 1. distance matrix -------------------------------------------------
    print("\n[1/9] Building flight-distance matrix (no-fly-zone aware)...")
    flight_matrix, detours = build_flight_distance_matrix(locations, city.no_fly_zones, drone_config=city.drone)
    if detours:
        print(f"  {len(detours)} directed leg(s) required a no-fly detour:")
        for (i, j) in detours:
            print(f"    {locations[i].name.split(',')[0]} -> {locations[j].name.split(',')[0]}")
    else:
        print("  No direct legs crossed a no-fly zone.")

    osmnx_matrix = build_osmnx_reference_matrix(locations, city.osmnx_network_type) if use_osmnx else None

    # --- 2. from-scratch solver (multi-heuristic) ---------------------------
    print("\n[2/9] Solving with from-scratch heuristics (NN + Clarke-Wright + Urgency-NN)...")
    print("  Trying all three construction heuristics with 2-opt + or-opt improvement...")

    # Show individual heuristic results for comparison
    for name in _CONSTRUCTION_HEURISTICS:
        routes = solve_vrp_with_heuristic(locations, flight_matrix, city.drone,
                                           construction=name, use_two_opt=True, use_or_opt=True)
        dist = total_distance(routes, flight_matrix)
        short = name.replace("_", " ").title()
        print(f"    {short:35s} -> {dist:.2f} km ({len(routes)} routes)")

    scratch_routes = solve_vrp_from_scratch(locations, flight_matrix, city.drone)
    scratch_total = total_distance(scratch_routes, flight_matrix)
    print_routes(scratch_routes, locations, flight_matrix, "Best from-scratch solution")

    # --- 3. OR-Tools benchmark -----------------------------------------------
    print("\n[3/9] Solving the identical problem with OR-Tools (benchmark)...")
    ortools_routes = solve_vrp_with_ortools(locations, flight_matrix, city.drone)
    if ortools_routes is not None:
        ortools_total = total_distance(ortools_routes, flight_matrix)
        print_routes(ortools_routes, locations, flight_matrix, "OR-Tools solution")
        gap_pct = 100.0 * (scratch_total - ortools_total) / ortools_total
        print(f"\n  From-scratch total: {scratch_total:.2f} km")
        print(f"  OR-Tools total:     {ortools_total:.2f} km")
        print(f"  Gap: {gap_pct:+.1f}% ({'from-scratch is worse' if gap_pct > 0 else 'from-scratch matched or beat OR-Tools'})")
    else:
        print(f"  Skipped (see message above). From-scratch total distance: "
              f"{scratch_total:.2f} km")

    # --- 4. time windows ------------------------------------------------------
    print("\n[4/9] Checking delivery-window urgency against the best solution...")
    tw_report = check_time_windows(scratch_routes, locations, flight_matrix, city.drone)
    if tw_report:
        for route_idx, etas in tw_report.items():
            for eta in etas:
                if eta.window_minutes is not None:
                    status = "MISSED" if eta.missed_window else "on time"
                    print(f"  Route {route_idx + 1} -> {eta.location_name}: "
                          f"ETA {eta.eta_minutes} min / window {eta.window_minutes} min [{status}]")
    else:
        print("  No locations in this run have a delivery-window constraint set.")

    # --- 5. cold chain --------------------------------------------------------
    print("\n[5/9] Checking cold-chain compliance (cumulative refrigeration time)...")
    cc_report = check_cold_chain_all_routes(scratch_routes, locations, flight_matrix, city.drone)
    if cc_report:
        for route_idx, results in cc_report.items():
            for r in results:
                status = "VIOLATED" if r.violated else "OK"
                print(f"  Route {route_idx + 1} -> {r.location_name}: "
                      f"exposure {r.actual_exposure_minutes} min / "
                      f"limit {r.cold_chain_limit_minutes} min [{status}]")
    else:
        print("  No cold-chain items on this run's routes.")

    # --- 6. landing-zone safety check ------------------------------------------
    print("\n[6/9] Running YOLOv8 landing-zone safety check...")
    safety_results = check_landing_zones(scratch_routes, locations)
    if safety_results:
        for r in safety_results:
            verdict = "SAFE" if r.is_safe else "UNSAFE -- skip/replan"
            print(f"  {r.location_name}: {verdict} ({r.reason})")
    else:
        print("  No locations with a configured landing_zone_image were on this run's routes, "
              "or the safety check is unavailable in this environment (see message above).")

    # --- 7. animated map --------------------------------------------------------
    print("\n[7/9] Rendering animated folium map...")
    map_path = render_map(scratch_routes, locations, city.drone, city.no_fly_zones, detours)
    if map_path:
        print(f"  Saved to {map_path}")

    # --- 8. waypoint export -------------------------------------------------------
    print("\n[8/9] Exporting QGC WPL 110 waypoint files (with A* detour waypoints)...")
    wpl_paths = export_all_routes(scratch_routes, locations, detours=detours, matrix=flight_matrix, drone=city.drone)
    for p in wpl_paths:
        print(f"  Saved {p}")

    # --- 9. static plots ----------------------------------------------------------
    print("\n[9/9] Generating static matplotlib plots...")
    try:
        from plots import plot_routes, plot_heuristic_comparison, save_all_plots
        from experiments import experiment_heuristic_comparison
        exp1 = experiment_heuristic_comparison(city)
        save_all_plots(city, {"heuristic_comparison": exp1})
    except Exception as e:
        print(f"  Skipped ({e})")

    # --- optional: full experiment suite -----------------------------------------
    if run_experiments:
        print("\n" + "=" * 60)
        print("  RUNNING FULL EXPERIMENT SUITE")
        print("=" * 60)
        try:
            from experiments import run_all_experiments
            run_all_experiments(city)
        except Exception as e:
            print(f"  Experiment suite failed: {e}")

    print("\nDone.")
    return {
        "flight_matrix": flight_matrix,
        "osmnx_matrix": osmnx_matrix,
        "detours": detours,
        "scratch_routes": scratch_routes,
        "ortools_routes": ortools_routes,
        "safety_results": safety_results,
        "map_path": map_path,
        "waypoint_paths": wpl_paths,
    }


def _get_city(name: str) -> CityConfig:
    """Look up a city config by name."""
    if name in CITY_CONFIGS:
        return CITY_CONFIGS[name]
    available = ", ".join(CITY_CONFIGS.keys())
    print(f"[warning] Unknown city '{name}'. Available: {available}. "
          f"Falling back to jaipur_disaster.")
    return JAIPUR_DISASTER


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Drone medical-supply delivery route optimizer — "
                    "disaster response and hospital logistics"
    )
    parser.add_argument("--city", default="jaipur_disaster",
                        help="Scenario to run (default: jaipur_disaster). "
                             f"Options: {', '.join(CITY_CONFIGS.keys())}")
    parser.add_argument("--from-json", dest="json_file", default=None,
                        help="Load scenario from a JSON file (works for ANY "
                             "location on Earth). See load_scenario.py for format.")
    parser.add_argument("--random-at", nargs=2, type=float, default=None,
                        metavar=("LAT", "LON"),
                        help="Generate and solve a random scenario centered at "
                             "these GPS coordinates. Example: --random-at 40.7580 -73.9855")
    parser.add_argument("--no-osmnx", action="store_true",
                        help="Skip OSMnx road-network reference matrix")
    parser.add_argument("--experiments", action="store_true",
                        help="Run the full experiment suite after the main pipeline")
    args = parser.parse_args()

    if args.json_file:
        from load_scenario import load_scenario_from_json
        city = load_scenario_from_json(args.json_file)
    elif args.random_at:
        from load_scenario import generate_random_scenario
        city = generate_random_scenario(args.random_at[0], args.random_at[1],
                                         name=f"Random Scenario ({args.random_at[0]}, {args.random_at[1]})")
    else:
        city = _get_city(args.city)
    run(city, use_osmnx=not args.no_osmnx, run_experiments=args.experiments)
