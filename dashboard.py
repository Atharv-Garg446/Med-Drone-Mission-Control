"""
dashboard.py
============
Interactive Streamlit dashboard for the drone medical-supply route optimizer.

This is the visual demo for interviews and portfolio presentations. It lets
the viewer:
  - Switch between disaster/city scenarios (Jaipur Flood, Chennai Cyclone, etc.)
  - Adjust drone parameters (capacity, range, speed) with sliders
  - Compare construction heuristics side-by-side
  - See the route map update in real time
  - Inject a Temporary Flight Restriction (TFR) and watch instant re-solve
  - Inject an emergency critical delivery and watch instant replanning
  - View cold-chain compliance status
  - Run experiments and see results as charts

Run with:  streamlit run dashboard.py
"""

import streamlit as st
import sys
import os
import time

# Ensure imports work regardless of working directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import CityConfig, DroneConfig, Location, CITY_CONFIGS
from distance_matrix import build_flight_distance_matrix
from vrp_scratch import (
    solve_vrp_from_scratch, solve_vrp_with_heuristic,
    total_distance, _route_distance, _route_demand,
    _CONSTRUCTION_HEURISTICS,
)
from time_windows import check_time_windows
from cold_chain import check_cold_chain_all_routes
from experiments import (
    run_solver_variant, experiment_heuristic_comparison,
    experiment_constraint_sensitivity, experiment_nofly_cost,
    experiment_replanning, SolverResult,
)

# ---------------------------------------------------------------------------
# Page Config
# ---------------------------------------------------------------------------

st.set_page_config(
    page_title="Drone Medical Router — Disaster Response",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded",
)


# ---------------------------------------------------------------------------
# Sidebar
# ---------------------------------------------------------------------------

st.sidebar.title("🚁 Drone Medical Router")
st.sidebar.markdown("*Disaster-Response Medical Aid Delivery*")
st.sidebar.markdown("---")

# City / scenario selection
city_display_names = {k: v.name for k, v in CITY_CONFIGS.items()}
selected_city_name = st.sidebar.selectbox(
    "📍 Scenario",
    list(CITY_CONFIGS.keys()),
    index=0,
    format_func=lambda x: city_display_names.get(x, x.title()),
)
base_city = CITY_CONFIGS[selected_city_name]

# Show scenario description
if base_city.scenario_description:
    st.sidebar.info(base_city.scenario_description)

st.sidebar.markdown("### 🔧 Drone Parameters")
capacity_kg = st.sidebar.slider(
    "Payload capacity (kg)", 5.0, 20.0, base_city.drone.capacity_kg, 0.5
)
max_range_km = st.sidebar.slider(
    "Max range (km)", 15.0, 80.0, base_city.drone.max_range_km, 1.0
)
cruise_speed = st.sidebar.slider(
    "Cruise speed (km/h)", 20.0, 80.0, base_city.drone.cruise_speed_kmh, 5.0
)

drone = DroneConfig(
    capacity_kg=capacity_kg,
    max_range_km=max_range_km,
    cruise_speed_kmh=cruise_speed,
)

st.sidebar.markdown("### 🧠 Construction Heuristic")
selected_heuristic = st.sidebar.selectbox(
    "Primary heuristic",
    list(_CONSTRUCTION_HEURISTICS.keys()),
    index=2,  # default to clarke_wright
    format_func=lambda x: x.replace("_", " ").title()
)

use_two_opt = st.sidebar.checkbox("Apply 2-opt (intra-route)", value=True)
use_or_opt = st.sidebar.checkbox("Apply or-opt (inter-route)", value=True)


# ---------------------------------------------------------------------------
# TFR (Temporary Flight Restriction) injection
# ---------------------------------------------------------------------------

st.sidebar.markdown("---")
st.sidebar.markdown("### 🚨 Event Injection")

inject_tfr = st.sidebar.checkbox("Inject TFR (no-fly zone)", value=False)
tfr_zone = None
if inject_tfr:
    st.sidebar.caption(
        "Simulates a Temporary Flight Restriction — an area the drone must "
        "now avoid. The solver re-runs instantly with the new obstacle."
    )
    tfr_lat = st.sidebar.number_input("TFR center lat", value=base_city.depot.lat - 0.02, format="%.4f")
    tfr_lon = st.sidebar.number_input("TFR center lon", value=base_city.depot.lon + 0.01, format="%.4f")
    tfr_size = st.sidebar.slider("TFR radius (approx degrees)", 0.005, 0.03, 0.01, 0.002)
    tfr_zone = [
        (tfr_lat - tfr_size, tfr_lon - tfr_size),
        (tfr_lat - tfr_size, tfr_lon + tfr_size),
        (tfr_lat + tfr_size, tfr_lon + tfr_size),
        (tfr_lat + tfr_size, tfr_lon - tfr_size),
    ]

inject_emergency = st.sidebar.checkbox("Inject emergency delivery", value=False)
emergency_loc = None
if inject_emergency:
    st.sidebar.caption(
        "Adds a new critical delivery point mid-plan. The solver re-runs "
        "instantly — demonstrating real-time replanning capability."
    )
    emg_lat = st.sidebar.number_input("Emergency lat", value=base_city.depot.lat - 0.01, format="%.4f")
    emg_lon = st.sidebar.number_input("Emergency lon", value=base_city.depot.lon - 0.015, format="%.4f")
    emg_demand = st.sidebar.slider("Emergency demand (kg)", 1.0, 8.0, 3.0, 0.5)
    emergency_loc = Location(
        id=len(base_city.deliveries) + 1,
        name="EMERGENCY — Injected Critical Delivery",
        lat=emg_lat,
        lon=emg_lon,
        demand_kg=emg_demand,
        urgency="critical",
        window_minutes=15,
        cold_chain_limit_minutes=30,
    )


# ---------------------------------------------------------------------------
# Build the active city config (with injections applied)
# ---------------------------------------------------------------------------

active_deliveries = list(base_city.deliveries)
if emergency_loc:
    active_deliveries.append(emergency_loc)

active_no_fly = list(base_city.no_fly_zones)
if tfr_zone:
    active_no_fly.append(tfr_zone)

city = CityConfig(
    name=base_city.name,
    depot=base_city.depot,
    deliveries=active_deliveries,
    no_fly_zones=active_no_fly,
    drone=drone,
    osmnx_network_type=base_city.osmnx_network_type,
    scenario_description=base_city.scenario_description,
)


# ---------------------------------------------------------------------------
# Core computation (cached by inputs)
# ---------------------------------------------------------------------------

@st.cache_data(show_spinner="Building distance matrix...")
def get_matrix(locations_key, no_fly_key):
    """Cache the distance matrix by location + no-fly zone fingerprint."""
    locs = city.all_locations
    matrix, detours = build_flight_distance_matrix(locs, city.no_fly_zones, drone_config=city.drone)
    return matrix, detours


# Create hashable keys for caching
loc_key = tuple((l.id, l.lat, l.lon) for l in city.all_locations)
nfz_key = tuple(tuple(tuple(p) for p in z) for z in city.no_fly_zones)

matrix, detours = get_matrix(loc_key, nfz_key)
locations = city.all_locations


# ---------------------------------------------------------------------------
# Main content
# ---------------------------------------------------------------------------

st.title(f"🚁 {city.name}")

# Injection banners
if inject_tfr:
    st.warning("⚡ **TFR ACTIVE** — A Temporary Flight Restriction has been injected. "
               "The solver has re-routed around it instantly.")
if inject_emergency:
    st.error("🚨 **EMERGENCY DELIVERY INJECTED** — A new critical delivery point has been "
             "added. The solver has replanned instantly.")

# Solve
try:
    t0 = time.perf_counter()
    routes = solve_vrp_with_heuristic(
        locations, matrix, drone,
        construction=selected_heuristic,
        use_two_opt=use_two_opt,
        use_or_opt=use_or_opt,
    )
    solve_ms = (time.perf_counter() - t0) * 1000
    total_km = total_distance(routes, matrix)
    solve_ok = True
except (ValueError, RuntimeError) as e:
    st.error(f"Solver failed: {e}")
    solve_ok = False
    routes = []
    total_km = 0
    solve_ms = 0

if solve_ok:
    # --- Metrics row ---
    col1, col2, col3, col4, col5 = st.columns(5)
    col1.metric("Total Distance", f"{total_km:.1f} km")
    col2.metric("Routes (Trips)", len(routes))
    col3.metric("Delivery Stops", sum(len(r) - 2 for r in routes))
    col4.metric("Detoured Legs", len(detours))
    col5.metric("Solve Time", f"{solve_ms:.0f} ms")

    # --- Route map ---
    st.subheader("📍 Route Map")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from plots import plot_routes

        title_suffix = ""
        if inject_tfr:
            title_suffix += " [TFR Active]"
        if inject_emergency:
            title_suffix += " [Emergency]"

        fig, ax = plot_routes(
            routes, locations, city.no_fly_zones, detours,
            title=f"{selected_heuristic.replace('_', ' ').title()} — {total_km:.1f} km total{title_suffix}"
        )
        st.pyplot(fig)
        plt.close(fig)
    except Exception as e:
        st.warning(f"Could not render map: {e}")

    # --- Route details ---
    st.subheader("📋 Route Details")
    for i, route in enumerate(routes):
        names = " → ".join(locations[n].name.split(",")[0] for n in route)
        dist = sum(matrix[route[k]][route[k + 1]] for k in range(len(route) - 1))
        demand = sum(locations[n].demand_kg for n in route[1:-1])
        battery_pct = max(0, 100 * (1 - dist / drone.max_range_km))
        st.markdown(
            f"**Route {i+1}**: {names}  \n"
            f"📏 {dist:.1f} km &nbsp; 📦 {demand:.1f} kg &nbsp; "
            f"🔋 {battery_pct:.0f}% remaining &nbsp; 🏥 {len(route)-2} stops"
        )

    # --- Time windows ---
    tw_report = check_time_windows(routes, locations, matrix, drone)
    if tw_report:
        st.subheader("⏱️ Delivery Windows")
        for route_idx, etas in tw_report.items():
            for eta in etas:
                if eta.window_minutes is not None:
                    status = "❌ MISSED" if eta.missed_window else "✅ On time"
                    st.markdown(
                        f"  Route {route_idx+1} → **{eta.location_name.split(',')[0]}**: "
                        f"ETA {eta.eta_minutes:.0f} min / "
                        f"Window {eta.window_minutes:.0f} min — {status}"
                    )

    # --- Cold-chain compliance ---
    cc_report = check_cold_chain_all_routes(routes, locations, matrix, drone)
    if cc_report:
        st.subheader("🧊 Cold-Chain Compliance")
        for route_idx, results in cc_report.items():
            for r in results:
                status = "❌ VIOLATED" if r.violated else "✅ OK"
                pct = 100 * r.actual_exposure_minutes / r.cold_chain_limit_minutes
                st.markdown(
                    f"  Route {route_idx+1} → **{r.location_name.split(',')[0]}**: "
                    f"exposure {r.actual_exposure_minutes:.0f} min / "
                    f"limit {r.cold_chain_limit_minutes:.0f} min "
                    f"({pct:.0f}%) — {status}"
                )


# ---------------------------------------------------------------------------
# Experiments Section
# ---------------------------------------------------------------------------

st.markdown("---")
st.header("🧪 Experiments")

tab1, tab2, tab3, tab4 = st.tabs([
    "Heuristic Comparison",
    "Range Sensitivity",
    "No-Fly Zone Cost",
    "Emergency Replanning",
])

with tab1:
    st.subheader("Which construction heuristic works best?")
    if st.button("Run Comparison", key="exp1"):
        with st.spinner("Running all heuristic combinations..."):
            exp1 = experiment_heuristic_comparison(city)

        results = exp1["results"]
        rows = []
        for r in results:
            opts = []
            if r.use_two_opt:
                opts.append("2-opt")
            if r.use_or_opt:
                opts.append("or-opt")
            rows.append({
                "Construction": r.construction.replace("_", " ").title(),
                "Improvement": " + ".join(opts) if opts else "None",
                "Distance (km)": round(r.total_km, 2),
                "Routes": r.num_routes,
                "Time (ms)": round(r.solve_time_ms, 1),
                "Windows Met": f"{r.windows_checked - r.windows_missed}/{r.windows_checked}",
            })
        st.dataframe(rows, use_container_width=True)

        best = min(results, key=lambda r: r.total_km)
        worst = max(results, key=lambda r: r.total_km)
        gap = worst.total_km - best.total_km
        st.success(
            f"Best: **{best.construction.replace('_', ' ').title()}** "
            f"({best.total_km:.1f} km) — "
            f"Gap between best and worst: **{gap:.1f} km ({100*gap/best.total_km:.1f}%)**"
        )

        try:
            from plots import plot_heuristic_comparison
            fig, _ = plot_heuristic_comparison(results)
            st.pyplot(fig)
            plt.close(fig)
        except Exception:
            pass

with tab2:
    st.subheader("How does battery range affect the solution?")
    if st.button("Run Sensitivity Analysis", key="exp2"):
        with st.spinner("Testing range variations..."):
            exp2 = experiment_constraint_sensitivity(city)

        feasible = [r for r in exp2["results"] if r["feasible"]]
        infeasible = [r for r in exp2["results"] if not r["feasible"]]

        rows = []
        for r in exp2["results"]:
            rows.append({
                "Range (km)": round(r["range_km"], 1),
                "Factor": f"{r['range_factor']:.0%}",
                "Distance (km)": round(r["total_km"], 1) if r["feasible"] else "—",
                "Routes": r["num_routes"] if r["feasible"] else "—",
                "Feasible": "✅" if r["feasible"] else "❌",
            })
        st.dataframe(rows, use_container_width=True)

        if infeasible:
            st.warning(
                f"⚠️ {len(infeasible)} range value(s) produced infeasible solutions — "
                f"at least one delivery is unreachable at that range."
            )

        try:
            from plots import plot_sensitivity
            fig = plot_sensitivity(exp2["results"], exp2["base_range_km"])
            st.pyplot(fig)
            plt.close(fig)
        except Exception:
            pass

with tab3:
    st.subheader("What distance penalty do no-fly zones impose?")
    if st.button("Run No-Fly Zone Analysis", key="exp4"):
        with st.spinner("Computing with and without no-fly zones..."):
            exp4 = experiment_nofly_cost(city)

        col1, col2, col3 = st.columns(3)
        col1.metric("With No-Fly Zones", f"{exp4['with_nfz'].total_km:.1f} km")
        col2.metric("Without No-Fly Zones", f"{exp4['without_nfz'].total_km:.1f} km")
        col3.metric("Penalty", f"+{exp4['distance_penalty_km']:.1f} km ({exp4['penalty_pct']:.1f}%)")

        st.info(
            f"**{exp4['num_detoured_legs']}** out of {len(locations)**2 - len(locations)} directed legs "
            f"required A* detours around {exp4['num_zones']} no-fly zone(s)."
        )

with tab4:
    st.subheader("What happens when an emergency delivery is added?")
    if st.button("Simulate Emergency", key="exp3"):
        with st.spinner("Re-solving with emergency delivery..."):
            exp3 = experiment_replanning(city)

        col1, col2 = st.columns(2)
        with col1:
            st.markdown("**Original Plan**")
            st.metric("Distance", f"{exp3['original'].total_km:.1f} km")
            st.metric("Routes", exp3['original'].num_routes)
        with col2:
            st.markdown("**After Emergency Added**")
            st.metric("Distance", f"{exp3['replanned'].total_km:.1f} km",
                      delta=f"+{exp3['distance_increase_km']:.1f} km")
            st.metric("Routes", exp3['replanned'].num_routes,
                      delta=exp3['route_change'] if exp3['route_change'] != 0 else None)

        st.info(
            f"Replanning took **{exp3['replan_time_ms']:.0f} ms** — fast enough for "
            f"real-time re-solving when a new delivery request arrives."
        )

        emg = exp3['emergency_location']
        st.markdown(
            f"Emergency delivery: **{emg.name}** at ({emg.lat:.4f}, {emg.lon:.4f}), "
            f"{emg.demand_kg} kg, {emg.urgency} priority, "
            f"{emg.window_minutes} min window"
        )


# ---------------------------------------------------------------------------
# Footer
# ---------------------------------------------------------------------------

st.markdown("---")
st.caption(
    "Drone Medical-Supply Route Optimizer — Disaster Response & Hospital Logistics. "
    "Built as a continuation of a DronAid internship, focusing on the computational, "
    "algorithmic, and autonomous-systems side of medical drone delivery. "
    "See the README for architecture, design decisions, and scope boundaries."
)
