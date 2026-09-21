"""
plots.py
========
Matplotlib-based static visualizations for routes, experiments, and
architecture diagrams. Used by the Streamlit dashboard and for README
screenshots.

All plots use matplotlib (no external mapping tiles, no internet needed)
so they work in any environment. The route map is a schematic -- not a
satellite image -- but it clearly shows the spatial structure: depot,
delivery points, no-fly zones, routes, and A* detours.
"""

import math
import os
from typing import List, Dict, Tuple, Optional

from config import Location, DroneConfig, CityConfig
from geo_utils import haversine_km


# Route colors matching the folium visualization
ROUTE_COLORS = ["#2166AC", "#B2182B", "#1A9850", "#762A83", "#E08214", "#4393C3",
                "#D6604D", "#4DAC26", "#80CDC1", "#FDB863"]

URGENCY_MARKERS = {"critical": "^", "urgent": "s", "routine": "o"}
URGENCY_COLORS = {"critical": "#D32F2F", "urgent": "#F57C00", "routine": "#1976D2"}


def plot_routes(
    routes: List[List[int]],
    locations: List[Location],
    no_fly_zones: List[List[Tuple[float, float]]],
    detours: Dict[Tuple[int, int], list],
    title: str = "Drone Delivery Routes",
    ax=None,
    show_legend: bool = True,
):
    """Plot routes on a schematic map with no-fly zones and detours."""
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.patches import Polygon as MplPolygon
    from matplotlib.collections import PatchCollection

    if ax is None:
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    else:
        fig = ax.figure

    # --- No-fly zones as shaded red polygons ---
    for zone in no_fly_zones:
        polygon = MplPolygon(
            [(lon, lat) for lat, lon in zone],
            closed=True, facecolor="#FFCDD2", edgecolor="#D32F2F",
            linewidth=1.5, alpha=0.5, linestyle="--",
            label="No-fly zone",
        )
        ax.add_patch(polygon)

    # --- Routes as colored lines ---
    for r_idx, route in enumerate(routes):
        color = ROUTE_COLORS[r_idx % len(ROUTE_COLORS)]
        # Build the full path including detour waypoints
        full_lats, full_lons = [], []
        for step in range(len(route) - 1):
            i, j = route[step], route[step + 1]
            if (i, j) in detours:
                wps = detours[(i, j)]
                for lat, lon in wps:
                    full_lats.append(lat)
                    full_lons.append(lon)
            else:
                full_lats.append(locations[i].lat)
                full_lons.append(locations[i].lon)
        # Add final point
        full_lats.append(locations[route[-1]].lat)
        full_lons.append(locations[route[-1]].lon)

        ax.plot([lon for lon in _lats_to_lons(full_lats, full_lons)],
                full_lats, color=color, linewidth=2, alpha=0.7,
                label=f"Route {r_idx + 1}", zorder=2)

    # --- Delivery locations ---
    for loc in locations[1:]:
        marker = URGENCY_MARKERS.get(loc.urgency, "o")
        color = URGENCY_COLORS.get(loc.urgency, "#1976D2")
        ax.scatter(loc.lon, loc.lat, marker=marker, c=color, s=80,
                   edgecolors="white", linewidths=0.5, zorder=4)
        ax.annotate(loc.name.split(",")[0], (loc.lon, loc.lat),
                    textcoords="offset points", xytext=(5, 5),
                    fontsize=6, color="#333333", zorder=5)

    # --- Depot ---
    depot = locations[0]
    ax.scatter(depot.lon, depot.lat, marker="*", c="black", s=200,
               edgecolors="white", linewidths=1, zorder=5)
    ax.annotate("DEPOT", (depot.lon, depot.lat),
                textcoords="offset points", xytext=(5, 8),
                fontsize=8, fontweight="bold", color="black", zorder=5)

    ax.set_xlabel("Longitude", fontsize=10)
    ax.set_ylabel("Latitude", fontsize=10)
    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    if show_legend:
        handles, labels = ax.get_legend_handles_labels()
        # Deduplicate "No-fly zone"
        seen = set()
        unique = [(h, l) for h, l in zip(handles, labels) if l not in seen and not seen.add(l)]
        if unique:
            ax.legend(*zip(*unique), loc="upper left", fontsize=7, framealpha=0.9)

    return fig, ax


def _lats_to_lons(lats, lons):
    """Helper to return lons (matplotlib wants x=lon, y=lat)."""
    return lons


def plot_heuristic_comparison(results, title="Construction Heuristic Comparison", ax=None):
    """Bar chart comparing heuristic + improvement combinations."""
    import matplotlib.pyplot as plt

    if ax is None:
        fig, ax = plt.subplots(1, 1, figsize=(12, 5))
    else:
        fig = ax.figure

    labels = []
    distances = []
    colors = []
    color_map = {
        "nearest_neighbor": "#2166AC",
        "urgency_nearest_neighbor": "#E08214",
        "clarke_wright": "#1A9850",
    }

    for r in results:
        opts = []
        if r.use_two_opt:
            opts.append("2opt")
        if r.use_or_opt:
            opts.append("oropt")
        opt_str = "+" + "+".join(opts) if opts else " only"
        short_name = r.construction.replace("_", " ").replace("nearest neighbor", "NN")
        short_name = short_name.replace("urgency NN", "Urgency NN")
        short_name = short_name.replace("clarke wright", "Clarke-Wright")
        labels.append(f"{short_name}\n{opt_str}")
        distances.append(r.total_km)
        colors.append(color_map.get(r.construction, "#888888"))

    bars = ax.bar(range(len(labels)), distances, color=colors, alpha=0.8, edgecolor="white")
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, fontsize=7, rotation=0, ha="center")
    ax.set_ylabel("Total Distance (km)", fontsize=10)
    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.grid(axis="y", alpha=0.3)

    # Annotate bars with exact values
    for bar, dist in zip(bars, distances):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.3,
                f"{dist:.1f}", ha="center", va="bottom", fontsize=7, fontweight="bold")

    # Mark the best
    best_idx = distances.index(min(distances))
    bars[best_idx].set_edgecolor("#1A9850")
    bars[best_idx].set_linewidth(2)

    return fig, ax


def plot_sensitivity(results, base_range_km, title="Flight Range Sensitivity", ax=None):
    """Line plot showing how distance and routes change with range."""
    import matplotlib.pyplot as plt

    if ax is None:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
    else:
        fig = ax.figure
        ax1 = ax
        ax2 = None

    feasible = [r for r in results if r["feasible"]]
    infeasible = [r for r in results if not r["feasible"]]

    ranges = [r["range_km"] for r in feasible]
    distances = [r["total_km"] for r in feasible]
    num_routes = [r["num_routes"] for r in feasible]

    ax1.plot(ranges, distances, "o-", color="#2166AC", linewidth=2, markersize=6)
    ax1.axvline(x=base_range_km, color="#888888", linestyle="--", alpha=0.5, label="Nominal range")
    for r in infeasible:
        ax1.axvline(x=r["range_km"], color="#D32F2F", linestyle=":", alpha=0.3)
    ax1.set_xlabel("Max Range (km)", fontsize=10)
    ax1.set_ylabel("Total Distance (km)", fontsize=10)
    ax1.set_title("Distance vs. Range", fontsize=11, fontweight="bold")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8)

    if ax2 is not None:
        ax2.plot(ranges, num_routes, "s-", color="#B2182B", linewidth=2, markersize=6)
        ax2.axvline(x=base_range_km, color="#888888", linestyle="--", alpha=0.5, label="Nominal range")
        ax2.set_xlabel("Max Range (km)", fontsize=10)
        ax2.set_ylabel("Number of Routes", fontsize=10)
        ax2.set_title("Routes vs. Range", fontsize=11, fontweight="bold")
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=8)
        ax2.yaxis.set_major_locator(plt.MaxNLocator(integer=True))

    plt.tight_layout()
    return fig


def save_all_plots(city: CityConfig, experiments: dict, output_dir: str = "output"):
    """Generate and save all static plots for the README/portfolio."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(output_dir, exist_ok=True)

    # Plot 1: Routes (best solution from heuristic comparison)
    exp1 = experiments.get("heuristic_comparison")
    if exp1:
        best_result = min(exp1["results"], key=lambda r: r.total_km)
        fig, _ = plot_routes(
            best_result.routes, city.all_locations,
            city.no_fly_zones, exp1["detours"],
            title=f"Best Route Solution — {city.name}"
        )
        fig.savefig(os.path.join(output_dir, "routes_map.png"), dpi=150, bbox_inches="tight")
        assets_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
        os.makedirs(assets_dir, exist_ok=True)
        fig.savefig(os.path.join(assets_dir, "routes_map.png"), dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"  Saved {output_dir}/routes_map.png and {assets_dir}/routes_map.png")

        # Plot 2: Heuristic comparison bar chart
        fig, _ = plot_heuristic_comparison(
            exp1["results"],
            title=f"Solver Comparison — {city.name}"
        )
        fig.savefig(os.path.join(output_dir, "heuristic_comparison.png"), dpi=150, bbox_inches="tight")
        fig.savefig(os.path.join(assets_dir, "heuristic_comparison.png"), dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"  Saved {output_dir}/heuristic_comparison.png and {assets_dir}/heuristic_comparison.png")

    # Plot 3: Sensitivity
    exp2 = experiments.get("constraint_sensitivity")
    if exp2:
        fig = plot_sensitivity(
            exp2["results"], exp2["base_range_km"],
            title=f"Flight Range Sensitivity — {city.name}"
        )
        fig.savefig(os.path.join(output_dir, "range_sensitivity.png"), dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"  Saved {output_dir}/range_sensitivity.png")

    print("  All plots saved.")
