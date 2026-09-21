"""
vrp_ortools.py
==============
Google OR-Tools solver benchmark.

Solves the CVRP using OR-Tools (Guided Local Search + Path Cheapest Arc)
as a benchmark for the custom solver. Uses the same distance matrix, 
demands, capacity, and range limits.

Requires: `pip install ortools`
"""

from typing import List
from config import Location, DroneConfig


def solve_vrp_with_ortools(
    locations: List[Location],
    matrix: List[List[float]],
    drone: DroneConfig,
    depot_index: int = 0,
    num_vehicles: int = 6,
    time_limit_seconds: int = 10,
):
    """Solve the same CVRP+range problem with OR-Tools.

    `num_vehicles` is an upper bound on how many separate depot-to-depot
    trips OR-Tools is allowed to use -- it's free to leave some unused.
    We default to a generous number (6) since drone routing is really
    "however many sequential trips one drone needs", and OR-Tools models
    that identically to "several vehicles used at once".

    Returns a list of routes in the same format as vrp_scratch.py:
    List[List[int]], each route a list of location indices, depot-to-depot.
    Returns None if ortools isn't installed, with an explanatory print --
    every other component of this project works fine without this file.
    """
    try:
        from ortools.constraint_solver import routing_enums_pb2
        from ortools.constraint_solver import pywrapcp
    except ImportError:
        print("[info] ortools not installed -- skipping the OR-Tools benchmark "
              "(`pip install ortools` to enable it). The from-scratch solver's "
              "result is still valid and usable on its own.")
        return None

    n = len(locations)

    # OR-Tools wants integer distances (it's a MIP-flavored solver under the
    # hood). We scale km to meters and round, which keeps sub-meter
    # precision loss irrelevant at drone-delivery distances.
    SCALE = 1000
    scaled_matrix = [[int(round(matrix[i][j] * SCALE)) for j in range(n)] for i in range(n)]
    demands = [int(round(loc.demand_kg * SCALE)) for loc in locations]  # same scale trick
    capacity = int(round(drone.capacity_kg * SCALE))
    max_range = int(round(drone.max_range_km * SCALE))

    manager = pywrapcp.RoutingIndexManager(n, num_vehicles, depot_index)
    routing = pywrapcp.RoutingModel(manager)

    # --- distance callback: how OR-Tools measures the cost of any edge ----
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return scaled_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # --- capacity dimension: enforces "sum of demand on a trip <= capacity"
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,                                   # no slack
        [capacity] * num_vehicles,            # same capacity per vehicle/trip
        True,                                 # start cumul at 0 for every vehicle
        "Capacity",
    )

    # --- distance dimension: enforces "total flight distance per trip <= max_range_km"
    # (this is what turns "vehicle capacity" into "vehicle capacity AND flight range")
    # Note: this benchmark enforces capacity + range only; the custom solver
    # additionally enforces cold-chain and time-window constraints.
    routing.AddDimension(
        transit_callback_index,
        0,                       # no slack
        max_range,               # max cumulative distance per vehicle
        True,                    # start cumul at 0 for every vehicle
        "Distance",
    )

    # --- search parameters: a reasonable, well-documented default recipe --
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(time_limit_seconds)

    solution = routing.SolveWithParameters(search_parameters)
    if solution is None:
        print("[warning] OR-Tools found no feasible solution within the time limit "
              "and constraints given (capacity / range too tight for this many "
              "vehicles?). Try raising num_vehicles or time_limit_seconds.")
        return None

    # --- unpack OR-Tools' internal solution into our plain route format ---
    routes = []
    for vehicle_id in range(num_vehicles):
        index = routing.Start(vehicle_id)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))  # closing depot node
        if len(route) > 2:  # skip vehicles OR-Tools chose not to use at all
            routes.append(route)

    return routes
