#!/usr/bin/env python3
"""
run_simulation.py
=================
ONE COMMAND to run the full simulation and open the interactive viewer.

Usage:
    python3 run_simulation.py                                    # Default Jaipur disaster
    python3 run_simulation.py --random-at 40.7580 -73.9855       # Random NYC scenario
    python3 run_simulation.py --from-json sample_data/sample_scenario.json
    python3 run_simulation.py --city chennai_flood --events 5

This script:
1. Runs the fleet simulation with your chosen scenario
2. Generates the interactive HTML viewer
3. Starts a local HTTP server (so map tiles load properly)
4. Opens your browser automatically

Press Ctrl+C to stop the server when done.
"""

import argparse
import http.server
import os
import random
import socketserver
import sys
import threading
import webbrowser

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import CITY_CONFIGS
from simulate_fleet import FleetSimulator, SimEvent, generate_random_events


def main():
    parser = argparse.ArgumentParser(description="Run drone simulation with interactive viewer")
    parser.add_argument("--city", default="jaipur_disaster", help="Built-in scenario name")
    parser.add_argument("--from-json", dest="json_file", help="Load scenario from JSON file")
    parser.add_argument("--random-at", nargs=2, type=float, metavar=("LAT", "LON"),
                        help="Generate random scenario at GPS coordinates")
    parser.add_argument("--events", type=int, default=3, help="Number of random events")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--port", type=int, default=8080, help="HTTP server port")
    parser.add_argument("--no-serve", action="store_true", help="Generate interactive simulation HTML without starting the web server")
    args = parser.parse_args()

    # Load scenario
    if args.json_file:
        from load_scenario import load_scenario_from_json
        city = load_scenario_from_json(args.json_file)
    elif args.random_at:
        from load_scenario import generate_random_scenario
        city = generate_random_scenario(args.random_at[0], args.random_at[1],
                                         name=f"Random ({args.random_at[0]:.4f}, {args.random_at[1]:.4f})")
    else:
        city = CITY_CONFIGS.get(args.city)
        if city is None:
            print(f"Unknown city '{args.city}'. Available: {', '.join(CITY_CONFIGS.keys())}")
            sys.exit(1)

    print(f"\n{'='*60}")
    print(f"  RUNNING SIMULATION: {city.name}")
    print(f"{'='*60}\n")

    # Generate events
    rng = random.Random(args.seed)
    events = generate_random_events(city, args.events, 3600, rng) if args.events > 0 else []

    # Run simulation
    sim = FleetSimulator(city, events=events, speed_multiplier=100)
    report = sim.run(max_ticks=3600)

    # Generate interactive HTML
    from build_interactive_sim import build_interactive_html
    os.makedirs("output", exist_ok=True)
    build_interactive_html(sim, "output/interactive_sim.html")

    if args.no_serve:
        print(f"Simulation complete. HTML viewer generated at: output/interactive_sim.html")
        return

    print(f"\n{'='*60}")
    print(f"  STARTING INTERACTIVE VIEWER")
    print(f"  Open: http://localhost:{args.port}/output/interactive_sim.html")
    print(f"  Press Ctrl+C to stop")
    print(f"{'='*60}\n")

    # Start HTTP server in background
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    handler = http.server.SimpleHTTPRequestHandler
    handler.log_message = lambda self, format, *a: None  # Suppress access logs

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", args.port), handler) as httpd:
        # Open browser
        url = f"http://localhost:{args.port}/output/interactive_sim.html"
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer stopped.")


if __name__ == "__main__":
    main()
