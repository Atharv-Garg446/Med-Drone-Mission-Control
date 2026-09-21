"""
experiments.py
==============
Systematic comparison of solver approaches across multiple scenarios.

This script runs the construction heuristics and local search algorithms
on controlled scenarios to measure performance differences.

Each experiment answers a concrete question:
  1. "Which construction heuristic is best, and by how much?"
  2. "How much do 2-opt and or-opt actually improve the initial solution?"
  3. "What happens when we tighten the range constraint?"
  4. "If an emergency delivery is added mid-mission, how does replanning compare?"
  5. "What distance penalty do the no-fly zones impose?"

Run standalone:  python3 experiments.py
Or from the dashboard:  the Streamlit app calls these functions interactively.
"""

import copy
import time
from typing import List, Dict, Any, Optional
from dataclasses import dataclass

from config import Location, DroneConfig, CityConfig, JAIPUR_DEMO
from distance_matrix import build_flight_distance_matrix
from vrp_scratch import (
    solve_vrp_with_heuristic, solve_vrp_from_scratch,
    total_distance, _route_distance, _route_demand,
    _CONSTRUCTION_HEURISTICS,
)
from time_windows import check_time_windows


@dataclass
class SolverResult:
    """Result of running one solver variant on one scenario."""
    construction: str
    use_two_opt: bool
    use_or_opt: bool
    routes: List[List[int]]
    total_km: float
    num_routes: int
    num_stops: int
    solve_time_ms: float
    windows_missed: int = 0       # how many time-windowed stops missed their deadline
    windows_checked: int = 0      # how many stops had a time window at all


def run_solver_variant(
    locations: List[Location],
    matrix: List[List[float]],
    drone: DroneConfig,
    construction: str,
    use_two_opt: bool,
    use_or_opt: bool,
) -> SolverResult:
    """Run a single solver configuration and measure its performance."""
    t0 = time.perf_counter()
    routes = solve_vrp_with_heuristic(
        locations, matrix, drone,
        construction=construction,
        use_two_opt=use_two_opt,
        use_or_opt=use_or_opt,
    )
    elapsed_ms = (time.perf_counter() - t0) * 1000

    # Check time windows
    tw_report = check_time_windows(routes, locations, matrix, drone)
    windows_missed = 0
    windows_checked = 0
    for etas in tw_report.values():
        for eta in etas:
            if eta.window_minutes is not None:
                windows_checked += 1
                if eta.missed_window:
                    windows_missed += 1

    return SolverResult(
        construction=construction,
        use_two_opt=use_two_opt,
        use_or_opt=use_or_opt,
        routes=routes,
        total_km=total_distance(routes, matrix),
        num_routes=len(routes),
        num_stops=sum(len(r) - 2 for r in routes),
        solve_time_ms=elapsed_ms,
        windows_missed=windows_missed,
        windows_checked=windows_checked,
    )


# ---------------------------------------------------------------------------
# Experiment 1: Construction Heuristic Comparison
# ---------------------------------------------------------------------------

def experiment_heuristic_comparison(city: CityConfig) -> Dict[str, Any]:
    """Compare all construction heuristics with and without improvement operators.

    Question: "Which construction heuristic produces the best initial solution,
    and how much do 2-opt and or-opt improve each one?"
    """
    locations = city.all_locations
    matrix, detours = build_flight_distance_matrix(locations, city.no_fly_zones, drone_config=city.drone)
    drone = city.drone

    configs = []
    for construction in _CONSTRUCTION_HEURISTICS:
        configs.append((construction, False, False))  # raw construction only
        configs.append((construction, True, False))    # + 2-opt
        configs.append((construction, True, True))     # + 2-opt + or-opt

    results = []
    for construction, two_opt, or_opt in configs:
        result = run_solver_variant(locations, matrix, drone, construction, two_opt, or_opt)
        results.append(result)

    return {
        "title": "Experiment 1: Construction Heuristic Comparison",
        "city": city.name,
        "results": results,
        "matrix": matrix,
        "detours": detours,
    }


# ---------------------------------------------------------------------------
# Experiment 2: Constraint Sensitivity
# ---------------------------------------------------------------------------

def experiment_constraint_sensitivity(city: CityConfig) -> Dict[str, Any]:
    """Vary flight range and observe how solutions change.

    Question: "What happens to total distance and number of routes as the
    drone's range gets tighter? Is there a critical threshold?"
    """
    locations = city.all_locations
    matrix, detours = build_flight_distance_matrix(locations, city.no_fly_zones, drone_config=city.drone)
    base_range = city.drone.max_range_km

    # Test range from 50% to 150% of nominal, in 10% steps
    range_factors = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.5]
    results = []

    for factor in range_factors:
        test_range = base_range * factor
        test_drone = DroneConfig(
            capacity_kg=city.drone.capacity_kg,
            max_range_km=test_range,
            cruise_speed_kmh=city.drone.cruise_speed_kmh,
        )
        try:
            result = run_solver_variant(
                locations, matrix, test_drone,
                construction="clarke_wright", use_two_opt=True, use_or_opt=True,
            )
            results.append({
                "range_km": test_range,
                "range_factor": factor,
                "total_km": result.total_km,
                "num_routes": result.num_routes,
                "feasible": True,
            })
        except (ValueError, RuntimeError):
            results.append({
                "range_km": test_range,
                "range_factor": factor,
                "total_km": None,
                "num_routes": None,
                "feasible": False,
            })

    return {
        "title": "Experiment 2: Flight Range Sensitivity",
        "city": city.name,
        "base_range_km": base_range,
        "results": results,
    }


# ---------------------------------------------------------------------------
# Experiment 3: Emergency Replanning
# ---------------------------------------------------------------------------

def experiment_replanning(city: CityConfig) -> Dict[str, Any]:
    """Simulate an emergency delivery added after initial planning.

    Question: "If a new critical delivery is added mid-mission, how much does
    the solution change? Is re-solving from scratch practical?"

    We solve the original problem, then add a new delivery point and re-solve,
    comparing the change in total distance, routes, and time windows.
    """
    locations = city.all_locations
    matrix, detours = build_flight_distance_matrix(locations, city.no_fly_zones, drone_config=city.drone)
    drone = city.drone

    # Solve original
    original = run_solver_variant(
        locations, matrix, drone,
        construction="clarke_wright", use_two_opt=True, use_or_opt=True,
    )

    # Add an emergency delivery: a new critical stop near the depot
    # (realistic: emergency blood unit needed at a nearby clinic)
    depot = city.depot
    emergency = Location(
        id=len(locations),
        name="Emergency Clinic (added)",
        lat=depot.lat + 0.015,  # ~1.7km north of depot
        lon=depot.lon - 0.010,  # ~0.9km west
        demand_kg=2.0,
        urgency="critical",
        window_minutes=15,
    )
    new_locations = locations + [emergency]
    new_matrix, new_detours = build_flight_distance_matrix(
        new_locations, city.no_fly_zones, drone_config=city.drone
    )

    # Re-solve with the new stop included
    replanned = run_solver_variant(
        new_locations, new_matrix, drone,
        construction="clarke_wright", use_two_opt=True, use_or_opt=True,
    )

    return {
        "title": "Experiment 3: Emergency Replanning",
        "city": city.name,
        "emergency_location": emergency,
        "original": original,
        "replanned": replanned,
        "distance_increase_km": replanned.total_km - original.total_km,
        "route_change": replanned.num_routes - original.num_routes,
        "replan_time_ms": replanned.solve_time_ms,
    }


# ---------------------------------------------------------------------------
# Experiment 4: No-Fly Zone Cost
# ---------------------------------------------------------------------------

def experiment_nofly_cost(city: CityConfig) -> Dict[str, Any]:
    """Measure the distance penalty of no-fly zones.

    Question: "How much extra distance do the no-fly zones force us to fly?"

    We solve once with no-fly zones (real distances including A* detours)
    and once without (pure straight-line distances). The difference is the
    operational cost of restricted airspace.
    """
    locations = city.all_locations
    drone = city.drone

    # With no-fly zones
    matrix_nfz, detours_nfz = build_flight_distance_matrix(locations, city.no_fly_zones, drone_config=city.drone)
    with_nfz = run_solver_variant(
        locations, matrix_nfz, drone,
        construction="clarke_wright", use_two_opt=True, use_or_opt=True,
    )

    # Without no-fly zones (straight-line everywhere)
    matrix_clear, _ = build_flight_distance_matrix(locations, [], drone_config=city.drone)
    without_nfz = run_solver_variant(
        locations, matrix_clear, drone,
        construction="clarke_wright", use_two_opt=True, use_or_opt=True,
    )

    return {
        "title": "Experiment 4: No-Fly Zone Cost Analysis",
        "city": city.name,
        "num_zones": len(city.no_fly_zones),
        "num_detoured_legs": len(detours_nfz),
        "with_nfz": with_nfz,
        "without_nfz": without_nfz,
        "distance_penalty_km": with_nfz.total_km - without_nfz.total_km,
        "penalty_pct": 100.0 * (with_nfz.total_km - without_nfz.total_km) / without_nfz.total_km
                       if without_nfz.total_km > 0 else 0,
    }


# ---------------------------------------------------------------------------
# Print Utilities
# ---------------------------------------------------------------------------

def _fmt_result(r: SolverResult) -> str:
    """Format a solver result as a compact summary line."""
    opts = []
    if r.use_two_opt:
        opts.append("2opt")
    if r.use_or_opt:
        opts.append("oropt")
    opt_str = "+".join(opts) if opts else "none"
    return (f"  {r.construction:30s} improve={opt_str:12s} -> "
            f"{r.total_km:7.2f} km  {r.num_routes} routes  "
            f"{r.solve_time_ms:6.1f} ms  "
            f"windows: {r.windows_checked - r.windows_missed}/{r.windows_checked} met")


def print_experiment_results(exp: Dict[str, Any]):
    """Pretty-print the results of any experiment."""
    print(f"\n{'=' * 72}")
    print(f"  {exp['title']}")
    print(f"  City: {exp.get('city', 'N/A')}")
    print(f"{'=' * 72}")

    if "results" in exp and isinstance(exp["results"], list):
        if isinstance(exp["results"][0], SolverResult):
            # Experiment 1: heuristic comparison
            for r in exp["results"]:
                print(_fmt_result(r))
            best = min(exp["results"], key=lambda r: r.total_km)
            worst = max(exp["results"], key=lambda r: r.total_km)
            print(f"\n  Best:  {best.total_km:.2f} km ({best.construction} + "
                  f"{'2opt+oropt' if best.use_or_opt else '2opt' if best.use_two_opt else 'none'})")
            print(f"  Worst: {worst.total_km:.2f} km ({worst.construction} + "
                  f"{'2opt+oropt' if worst.use_or_opt else '2opt' if worst.use_two_opt else 'none'})")
            print(f"  Gap:   {worst.total_km - best.total_km:.2f} km "
                  f"({100*(worst.total_km - best.total_km)/best.total_km:.1f}%)")
        else:
            # Experiment 2: constraint sensitivity
            print(f"  Base range: {exp.get('base_range_km', '?')} km\n")
            print(f"  {'Range (km)':>12s}  {'Factor':>8s}  {'Total km':>10s}  {'Routes':>8s}  {'Feasible':>10s}")
            print(f"  {'-'*12}  {'-'*8}  {'-'*10}  {'-'*8}  {'-'*10}")
            for r in exp["results"]:
                total_str = f"{r['total_km']:.2f}" if r['feasible'] else "---"
                routes_str = str(r['num_routes']) if r['feasible'] else "---"
                print(f"  {r['range_km']:12.1f}  {r['range_factor']:8.1%}  "
                      f"{total_str:>10s}  {routes_str:>8s}  "
                      f"{'YES' if r['feasible'] else 'NO':>10s}")

    elif "original" in exp and "replanned" in exp:
        # Experiment 3: replanning
        print(f"\n  Original solution:")
        print(f"    Distance: {exp['original'].total_km:.2f} km, "
              f"{exp['original'].num_routes} routes")
        print(f"\n  After adding emergency delivery at "
              f"({exp['emergency_location'].lat:.4f}, {exp['emergency_location'].lon:.4f}):")
        print(f"    Distance: {exp['replanned'].total_km:.2f} km, "
              f"{exp['replanned'].num_routes} routes")
        print(f"    Distance increase: +{exp['distance_increase_km']:.2f} km")
        print(f"    Route change: {'+' if exp['route_change'] >= 0 else ''}{exp['route_change']}")
        print(f"    Replan time: {exp['replan_time_ms']:.1f} ms")
        print(f"    Windows missed: {exp['replanned'].windows_missed}/{exp['replanned'].windows_checked}")

    elif "with_nfz" in exp and "without_nfz" in exp:
        # Experiment 4: no-fly zone cost
        print(f"\n  No-fly zones: {exp['num_zones']}")
        print(f"  Legs requiring A* detour: {exp['num_detoured_legs']}")
        print(f"\n  With no-fly zones:    {exp['with_nfz'].total_km:.2f} km, "
              f"{exp['with_nfz'].num_routes} routes")
        print(f"  Without no-fly zones: {exp['without_nfz'].total_km:.2f} km, "
              f"{exp['without_nfz'].num_routes} routes")
        print(f"  Distance penalty: +{exp['distance_penalty_km']:.2f} km "
              f"(+{exp['penalty_pct']:.1f}%)")


def run_all_experiments(city: CityConfig = None):
    """Run all experiments and print results. Returns the experiment dicts
    for programmatic use (dashboard, etc.)."""
    if city is None:
        city = JAIPUR_DEMO

    print(f"\nRunning experiments on {city.name}...")
    print(f"Locations: {len(city.deliveries)} deliveries + 1 depot")
    print(f"Drone: {city.drone.capacity_kg}kg capacity, "
          f"{city.drone.max_range_km}km range, "
          f"{city.drone.cruise_speed_kmh}km/h cruise")

    experiments = {}

    exp1 = experiment_heuristic_comparison(city)
    print_experiment_results(exp1)
    experiments["heuristic_comparison"] = exp1

    exp2 = experiment_constraint_sensitivity(city)
    print_experiment_results(exp2)
    experiments["constraint_sensitivity"] = exp2

    exp3 = experiment_replanning(city)
    print_experiment_results(exp3)
    experiments["replanning"] = exp3

    exp4 = experiment_nofly_cost(city)
    print_experiment_results(exp4)
    experiments["nofly_cost"] = exp4

    return experiments


if __name__ == "__main__":
    import sys
    # Allow specifying a city: python3 experiments.py bengaluru
    city_name = sys.argv[1] if len(sys.argv) > 1 else None
    city = JAIPUR_DEMO
    if city_name:
        try:
            from config import CITY_CONFIGS
            city = CITY_CONFIGS.get(city_name, JAIPUR_DEMO)
        except (ImportError, AttributeError):
            pass
    run_all_experiments(city)
