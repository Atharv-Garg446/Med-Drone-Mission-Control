# Med-Drone Mission Control

Route planning and fleet management software for medical drone delivery in disaster zones.

When floods, cyclones, or earthquakes cut off road access, this system figures out how to get medical supplies — vaccines, blood, emergency meds — from a hospital depot to relief camps and stranded clinics using a fleet of delivery drones.

It handles all the messy real-world constraints: limited flight range, cargo weight limits, no-fly zones, refrigeration deadlines for vaccines, and what to do when things go wrong mid-flight.

<p align="center">
  <img src="assets/routes_map.png" alt="Optimized drone routes with obstacle avoidance" width="550"/>
</p>
<p align="center">
  <em>Multi-drone routing across flooded Jaipur with A* detours around no-fly zones.</em>
</p>

---

## How It Works

The core problem is a **Capacitated Vehicle Routing Problem (CVRP)** — given N delivery locations and M drones with limited range and cargo space, find the shortest set of routes that covers everything.

Since CVRP is NP-hard, I built three construction heuristics from scratch and run all three, picking the best result:

1. **Nearest Neighbor** — greedy closest-unvisited, simple and fast
2. **Clarke-Wright Savings** — starts with individual round-trips, merges the ones that save the most distance
3. **Urgency-Weighted NN** — same as nearest neighbor but biases toward critical patients first

Each solution gets refined with **2-opt** (uncrosses paths within a route) and **or-opt** (moves stops between routes for better balance).

When a straight-line path crosses a no-fly zone, **A\* pathfinding** finds the shortest way around using a grid overlay with Haversine distances.

### Constraints enforced simultaneously:
- **Flight range** — total flight distance per trip can't exceed drone max (with reserve)
- **Cargo capacity** — total payload weight per trip
- **Cold chain** — cumulative time out of refrigeration since leaving the depot (not per-leg — a vaccine degrades continuously from launch)
- **Time windows** — delivery deadlines measured from when the request was created

<p align="center">
  <img src="assets/heuristic_comparison.png" alt="Solver benchmark comparison" width="700"/>
</p>
<p align="center">
  <em>Performance comparison across heuristics — no single one dominates on all inputs.</em>
</p>

---

## Dynamic Replanning

The fleet simulator runs missions forward in time and randomly injects disruptions — pop-up flight restrictions, new emergency patients, drone failures. When something changes mid-flight:

- Active drones evaluate from their **current GPS position** (no teleporting)
- Routes get re-solved in under **200ms** for the bundled scenarios (8–15 stops, 3–6 drones)
- If a drone can't make it back to depot, it enters emergency hold
- New emergency cargo physically originates at the depot — idle drones get dispatched immediately, or deliveries queue until a drone returns

The full re-solve approach works because with 8-15 stops and 3-6 drones, the solver is fast enough that patching would add complexity without meaningful speed benefit.

---

## Quick Start

```bash
git clone https://github.com/Atharv-Garg446/Med-Drone-Mission-Control.git
cd Med-Drone-Mission-Control

# Core pipeline — no pip installs needed
python3 main.py --no-osmnx

# Fleet simulation with random disruptions
python3 run_simulation.py --events 5
# Then open: http://localhost:8080/output/interactive_sim.html

# All 78 tests
python3 -m unittest discover -s tests -p "test_*.py" -v
```

**Optional dependencies** (install with `pip install -r requirements.txt`):

| Package | What it adds |
|---|---|
| `folium` | Interactive map visualization |
| `ortools` | Google OR-Tools for benchmark comparison |
| `ultralytics` | YOLOv8 landing zone obstruction screening |
| `matplotlib` | Static route plots |
| `streamlit` | Interactive dashboard |

---

## Architecture

```
config.py           → Scenario data: depot, deliveries, no-fly zones, drone specs
  ↓
distance_matrix.py  → Flight distances (straight-line + A* detours)
+ nofly_astar.py
  ↓
vrp_scratch.py      → 3 heuristics → 2-opt → or-opt → best solution
  ↓
cold_chain.py       → Cumulative refrigeration check
time_windows.py     → Delivery deadline check
yolo_safety.py      → Landing zone obstruction screening (pretrained YOLOv8)
  ↓
waypoint_export.py  → MAVLink .waypoints files (QGC WPL 110 for ArduPilot/PX4)
```

**Real-time layer:**
```
simulate_fleet.py → replan.py → re-solves CVRP for remaining deliveries
     ├── Pop-up TFR handling (escape + reroute)
     ├── Emergency cargo dispatch from depot
     ├── Drone failure redistribution
     └── Telemetry recording for visual replay
```

---

## Built-in Scenarios

**Jaipur Flood Response** — Monsoon flooding along the Dravyavati River. SMS Hospital on high ground serves as depot, 8 deliveries across relief camps.

**Chennai Cyclone Response** — Based on the 2015 South Indian floods. RGGGH hospital as depot, 7 deliveries across submerged southern neighborhoods.

**Jaipur Routine Delivery** — Same engine handling normal hospital restocks under calm conditions.

### Works anywhere:
```bash
python3 main.py --random-at 40.7580 -73.9855      # New York
python3 main.py --random-at 35.6586 139.7454       # Tokyo
python3 main.py --from-json your_scenario.json     # Your own city
```

Every algorithm takes a `CityConfig` object — nothing is hardcoded to any location.

---

## Real Drone Compatibility

The waypoint exporter generates `.waypoints` files in **QGC WPL 110** format, which is the standard mission format for ArduPilot and PX4 autopilots.

To test:
1. Run `python3 main.py --no-osmnx` → generates `output/*.waypoints`
2. Open [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) → Plan → File → Load
3. Connect to an ArduPilot SITL instance to watch the drone fly the mission

The exported missions include A* detour waypoints, so the drone actually flies around no-fly zones — not through them.

---

## Landing Zone Safety

Uses a pretrained YOLOv8n model (COCO, 80 classes) as a visual obstruction screen — checks aerial imagery of a landing zone for people, vehicles, animals, or debris before descent. The detection model is off-the-shelf, but the decision logic (which classes are unsafe, confidence thresholds, threat prioritization) is hand-written and unit-tested.

This is an operational screening layer, not a replacement for onboard hardware sensors.

---

## Testing

```bash
python3 -m unittest discover -s tests -p "test_*.py" -v   # 78 tests, zero pip installs
```

Coverage includes: Haversine geometry, A* pathfinding (including edge cases like pop-up TFR escape and dead-edge detection), all 3 CVRP heuristics with constraint checking, cold chain tracking, time window enforcement, YOLO decision logic, waypoint export validation, fleet simulation with disruptions, and scenario loading/validation.

---

## License

MIT
