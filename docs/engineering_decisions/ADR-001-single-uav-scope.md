# ADR-001: Primary Single-UAV Autonomy Scope vs. Multi-Drone Fleet Simulator

## Status
Accepted (2026-11-05)

## Context
The repository originally started as a Capacitated Vehicle Routing Problem (CVRP) solver and discrete-time multi-drone kinematic simulator (`vrp_scratch.py`, `simulate_fleet.py`, `replan.py`). Attempting to run simultaneous multi-UAV ArduPilot SITL instances while also developing live disaster ingestion, 4D airspace replanning, sensor-driven obstacle avoidance, and landing/delivery state machines on an 8 GB RAM Apple M3 machine diluted engineering depth and risked superficial integration.

## Decision
1. **Focus the primary operational flight stack (`sitl_mission_demo.py` and `flightstack/*`) on a single reference UAV (`JAIPUR_SINGLE_UAV`, `fleet_size = 1`).**
   - Ground the reference vehicle in `flightstack/vehicle_model.py` (`MedDrone-Hexa-16`: `5.5 kg` empty mass, `3.2 kg` 6S 16 Ah battery with `284.16 Wh` usable energy at `20%` reserve floor, `8.0 kg` max payload, `17.5 kg` MTOW, `10.0 m/s` cruise speed).
   - Decouple `sitl_mission_demo.py` from `solve_vrp_from_scratch` when `drone.fleet_size == 1` so the single-UAV workflow is self-contained.
2. **Preserve all multi-drone VRP and fleet-simulation modules (`simulate_fleet.py`, `vrp_scratch.py`, `replan.py`) intact** as a logistics-planning research component without forcing `fleet_size = 1` globally.

## Consequences
- **Positive:** Deep, verifiable integration on a single autopilot connection (`tcp:127.0.0.1:5760`), full mission readback verification, genuine C++ proximity raycasting (`PRX1_TYPE = 10`), and complete hover/land delivery lifecycles with zero regressions in existing VRP tests.
- **Trade-off:** Multi-UAV tactical deconfliction in SITL is deferred to future work and explicitly documented as out of scope for the current sprint.

