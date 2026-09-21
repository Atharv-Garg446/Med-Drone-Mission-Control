#!/usr/bin/env python3
"""
benchmark_replanning.py
=======================
Self-contained benchmark measuring flight distance-matrix generation
and dynamic CVRP replanning performance across 8-stop and 15-stop
scenarios. Evaluates cold-cache vs warm-cache execution times.

Usage:
    python3 benchmarks/benchmark_replanning.py
"""

import os
import sys
import time
from typing import List

# Ensure repository root is on sys.path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import JAIPUR_DISASTER, CHENNAI_FLOOD
from distance_matrix import build_flight_distance_matrix
from replan import replan_from_remaining
from nofly_astar import clear_grid_cache
from load_scenario import generate_random_scenario


def benchmark_matrix(scenario, label: str, trials: int = 5):
    # Cold cache run
    clear_grid_cache()
    t0 = time.perf_counter()
    build_flight_distance_matrix(scenario.all_locations, scenario.no_fly_zones, drone_config=scenario.drone)
    cold_sec = time.perf_counter() - t0

    # Warm cache runs
    warm_secs: List[float] = []
    for _ in range(trials):
        t0 = time.perf_counter()
        build_flight_distance_matrix(scenario.all_locations, scenario.no_fly_zones, drone_config=scenario.drone)
        warm_secs.append(time.perf_counter() - t0)

    avg_warm = sum(warm_secs) / len(warm_secs)
    min_warm = min(warm_secs)
    return cold_sec, avg_warm, min_warm


def benchmark_replan(depot, deliveries, zones, drone, label: str, trials: int = 5):
    # Cold cache run
    clear_grid_cache()
    t0 = time.perf_counter()
    replan_from_remaining(depot, deliveries, zones, drone)
    cold_sec = time.perf_counter() - t0

    # Warm cache runs
    warm_secs: List[float] = []
    for _ in range(trials):
        t0 = time.perf_counter()
        replan_from_remaining(depot, deliveries, zones, drone)
        warm_secs.append(time.perf_counter() - t0)

    avg_warm = sum(warm_secs) / len(warm_secs)
    min_warm = min(warm_secs)
    return cold_sec, avg_warm, min_warm


def main():
    print("=" * 68)
    print("  MED-DRONE MISSION CONTROL: REPLANNING & A* BENCHMARKS")
    print("=" * 68)
    print(f"Python: {sys.version.split()[0]} | Platform: {sys.platform}\n")

    # 1. Flight Distance Matrix Benchmarks
    print("--- 1. Flight Distance Matrix Construction ---")
    scenarios = [
        (JAIPUR_DISASTER, "Jaipur Disaster (9 locs, 18 detours)"),
        (CHENNAI_FLOOD, "Chennai Flood (8 locs, 12 detours)"),
    ]
    for sc, name in scenarios:
        cold, avg_w, min_w = benchmark_matrix(sc, name)
        print(f"{name:42} | Cold: {cold*1000:5.1f} ms | Warm (mean): {avg_w*1000:5.1f} ms (min: {min_w*1000:5.1f} ms)")

    # 2. 8-Stop Replan Benchmark (Jaipur Disaster)
    print("\n--- 2. 8-Stop Dynamic Replanning (Jaipur Disaster) ---")
    c8, w8_mean, w8_min = benchmark_replan(
        JAIPUR_DISASTER.depot, JAIPUR_DISASTER.deliveries,
        JAIPUR_DISASTER.no_fly_zones, JAIPUR_DISASTER.drone,
        "Jaipur 8-Stop"
    )
    print(f"{'8-Stop Jaipur Disaster Re-solve':42} | Cold: {c8*1000:5.1f} ms | Warm (mean): {w8_mean*1000:5.1f} ms (min: {w8_min*1000:5.1f} ms)")

    # 3. 15-Stop Replan Benchmark (across multiple random seeds)
    print("\n--- 3. 15-Stop Dynamic Replanning (Multi-Obstacle Scenarios) ---")
    seeds = [42, 10, 99, 777, 2026]
    cold_15, warm_15 = [], []
    for s in seeds:
        sc15 = generate_random_scenario(26.9124, 75.7873, num_deliveries=15, num_no_fly_zones=2, radius_km=10.0, seed=s)
        c15, w15_mean, w15_min = benchmark_replan(
            sc15.depot, sc15.deliveries, sc15.no_fly_zones, sc15.drone, f"15-Stop Seed {s}", trials=3
        )
        cold_15.append(c15)
        warm_15.append(w15_mean)
        print(f"  Seed {s:<5} (15 stops, 2 NFZs)              | Cold: {c15*1000:5.1f} ms | Warm (mean): {w15_mean*1000:5.1f} ms")

    avg_c15 = sum(cold_15) / len(cold_15)
    avg_w15 = sum(warm_15) / len(warm_15)
    print(f"{'15-Stop Multi-Seed Average':42} | Cold: {avg_c15*1000:5.1f} ms | Warm (mean): {avg_w15*1000:5.1f} ms")

    print("\n" + "=" * 68)
    print("  SUMMARY: MEASURED REPLANNING PERFORMANCE")
    print("=" * 68)
    print(f"  *  8-Stop Replan:  {w8_mean:.2f} s warm cache (~{w8_mean*1000:.0f} ms) / {c8:.2f} s cold cache (~{c8*1000:.0f} ms)")
    print(f"  * 15-Stop Replan:  {avg_w15:.2f} s warm cache (~{avg_w15*1000:.0f} ms) / {avg_c15:.2f} s cold cache (~{avg_c15*1000:.0f} ms)")
    print("=" * 68)


if __name__ == "__main__":
    main()
