#!/usr/bin/env python3
"""
visualize_sitl.py
=================
Generates an interactive HTML dashboard from ArduCopter SITL mission execution logs.

Visualizes:
  - Exact SITL autopilot EKF position trace vs static planner predictions.
  - Delivery stops, completed drop events, and active waypoint sequencing.
  - Static No-Fly Zones (red) and dynamically injected TFR / Obstacle exclusion zones (amber).
  - Telemetry HUD: Mode, Altitude, Groundspeed, Heading, Battery SoC %, EKF & GNSS health.
  - Interactive playback scrubber (1x, 2x, 5x, 10x) with event jump markers.

Usage:
    python3 visualize_sitl.py                            # Automatically finds latest sitl_run_*.jsonl
    python3 visualize_sitl.py output/sitl_run_XXX.jsonl -o output/sitl_flight_replay.html
"""

import argparse
import glob
import json
import os
import sys
from typing import Dict, List, Optional, Tuple


def find_latest_sitl_log(output_dir: str = "output") -> Optional[str]:
    logs = glob.glob(os.path.join(output_dir, "sitl_run_*.jsonl"))
    if not logs:
        return None
    return max(logs, key=os.path.getmtime)


def parse_sitl_log(log_path: str) -> dict:
    events = []
    telemetry_samples = []
    plan_meta = {}
    zones = []
    replans = []
    deliveries = []
    summary = {}

    with open(log_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue

            ev = rec.get("event")
            # EventLog writes t_rel_s, historical logs may write t
            t = float(rec.get("t_rel_s", rec.get("t", 0.0)))
            rec["t"] = t
            rec["t_rel_s"] = t
            events.append(rec)

            if ev == "planned":
                plan_meta = rec
            elif ev == "telemetry":
                tel = rec.get("telemetry", {})
                pos = tel.get("position")
                if pos:
                    sample = {
                        "t": t,
                        "t_rel_s": t,
                        "lat": pos.get("lat"),
                        "lon": pos.get("lon"),
                        "alt_rel_m": pos.get("alt_rel_m", 0.0),
                        "alt_amsl_m": pos.get("alt_amsl_m", 0.0),
                        "groundspeed_ms": pos.get("groundspeed_ms", 0.0),
                        "heading_deg": pos.get("heading_deg", 0.0),
                        "flight_mode": tel.get("flight_mode", "UNKNOWN"),
                        "armed": tel.get("armed", False),
                        "battery": tel.get("battery", {}),
                        "gnss": tel.get("gnss", {}),
                        "ekf": tel.get("ekf", {}),
                        "mission_current": tel.get("mission_current"),
                    }
                    telemetry_samples.append(sample)
            elif ev == "airspace_conflict":
                src = rec.get("source", {})
                poly = src.get("polygon")
                if poly:
                    zones.append({"type": "dynamic", "name": rec.get("reason", "TFR"), "polygon": poly, "t": t, "t_rel_s": t})
            elif ev == "obstacle_conflict":
                # Recorded dynamic obstacle alert
                pass
            elif ev == "mission_replanned":
                replans.append(rec)
            elif ev == "delivery_complete":
                deliveries.append(rec)
            elif ev == "summary":
                summary = rec

    return {
        "log_path": log_path,
        "plan_meta": plan_meta,
        "telemetry": telemetry_samples,
        "events": events,
        "zones": zones,
        "replans": replans,
        "deliveries": deliveries,
        "summary": summary,
    }


def generate_html_dashboard(data: dict, output_html_path: str) -> None:
    plan_meta = data["plan_meta"]
    scenario = plan_meta.get("scenario", "Medical Drone Mission")
    mission_items = plan_meta.get("mission", {}).get("items", [])
    telemetry = data["telemetry"]
    events = data["events"]

    # Filter out initial depot/home
    initial_home = [26.892, 75.826]  # default fallback
    for it in mission_items:
        if it.get("role") == "home" and it.get("lat") != 0.0:
            initial_home = [it["lat"], it["lon"]]
            break

    # Serialize data into JavaScript constants
    js_telemetry = json.dumps(telemetry)
    js_mission = json.dumps(mission_items)
    js_events = json.dumps(events)
    js_dynamic_zones = json.dumps(data["zones"])
    js_summary = json.dumps(data["summary"])
    js_home = json.dumps(initial_home)

    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>SITL Flight Replay — {scenario}</title>
  <!-- Leaflet CSS & JS -->
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY=" crossorigin="" />
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo=" crossorigin=""></script>
  <!-- Google Fonts -->
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=JetBrains+Mono:wght@400;600&display=swap" rel="stylesheet">
  <style>
    * {{
      box-sizing: border-box;
      margin: 0;
      padding: 0;
    }}
    body {{
      font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
      background: #0B0F19;
      color: #E2E8F0;
      height: 100vh;
      display: flex;
      flex-direction: column;
      overflow: hidden;
    }}
    header {{
      background: #111827;
      border-bottom: 1px solid #1F2937;
      padding: 10px 20px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      z-index: 1000;
    }}
    .brand {{
      display: flex;
      align-items: center;
      gap: 12px;
    }}
    .brand-icon {{
      font-size: 22px;
      background: #1E293B;
      padding: 6px;
      border-radius: 8px;
    }}
    .brand h1 {{
      font-size: 16px;
      font-weight: 700;
      color: #F8FAFC;
    }}
    .brand .badge {{
      font-size: 11px;
      font-weight: 600;
      padding: 2px 8px;
      border-radius: 4px;
      background: #0284C7;
      color: #FFFFFF;
      letter-spacing: 0.5px;
    }}
    .badge-sitl {{
      background: #059669 !important;
    }}
    .main-container {{
      flex: 1;
      position: relative;
      display: flex;
    }}
    #map {{
      width: 100%;
      height: 100%;
      background: #0B0F19;
    }}
    /* Floating HUD */
    .hud-panel {{
      position: absolute;
      top: 16px;
      right: 16px;
      width: 320px;
      background: rgba(15, 23, 42, 0.90);
      backdrop-filter: blur(12px);
      -webkit-backdrop-filter: blur(12px);
      border: 1px solid rgba(51, 65, 85, 0.8);
      border-radius: 12px;
      padding: 16px;
      box-shadow: 0 10px 25px -5px rgba(0, 0, 0, 0.5);
      z-index: 1000;
      font-size: 12px;
    }}
    .hud-title {{
      font-size: 11px;
      text-transform: uppercase;
      font-weight: 700;
      letter-spacing: 1px;
      color: #94A3B8;
      margin-bottom: 12px;
      border-bottom: 1px solid #1E293B;
      padding-bottom: 6px;
      display: flex;
      justify-content: space-between;
    }}
    .hud-grid {{
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 8px 12px;
    }}
    .hud-metric {{
      display: flex;
      flex-direction: column;
    }}
    .hud-metric .label {{
      font-size: 10px;
      color: #64748B;
      font-weight: 500;
    }}
    .hud-metric .val {{
      font-family: 'JetBrains Mono', monospace;
      font-size: 14px;
      font-weight: 600;
      color: #38BDF8;
    }}
    .hud-metric .val-mode {{
      color: #34D399;
      font-weight: 700;
    }}
    .hud-metric .val-warn {{
      color: #F87171;
    }}
    /* Bottom Controls */
    .controls-panel {{
      background: #111827;
      border-top: 1px solid #1F2937;
      padding: 12px 20px;
      display: flex;
      flex-direction: column;
      gap: 10px;
      z-index: 1000;
    }}
    .scrub-row {{
      display: flex;
      align-items: center;
      gap: 14px;
    }}
    .scrub-row input[type="range"] {{
      flex: 1;
      height: 6px;
      background: #334155;
      border-radius: 3px;
      outline: none;
      -webkit-appearance: none;
      cursor: pointer;
    }}
    .scrub-row input[type="range"]::-webkit-slider-thumb {{
      -webkit-appearance: none;
      width: 16px;
      height: 16px;
      background: #38BDF8;
      border-radius: 50%;
      cursor: pointer;
      box-shadow: 0 0 8px rgba(56, 189, 248, 0.6);
    }}
    .time-badge {{
      font-family: 'JetBrains Mono', monospace;
      font-size: 13px;
      color: #94A3B8;
      min-width: 65px;
    }}
    .playback-row {{
      display: flex;
      justify-content: space-between;
      align-items: center;
    }}
    .btn-group {{
      display: flex;
      gap: 8px;
      align-items: center;
    }}
    .btn {{
      background: #1E293B;
      border: 1px solid #334155;
      color: #F8FAFC;
      padding: 6px 12px;
      border-radius: 6px;
      font-size: 12px;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.15s ease;
    }}
    .btn:hover {{
      background: #334155;
      color: #38BDF8;
    }}
    .btn.active {{
      background: #0284C7;
      border-color: #38BDF8;
      color: #FFFFFF;
    }}
    .event-chips {{
      display: flex;
      gap: 6px;
      overflow-x: auto;
      max-width: 60%;
      padding-bottom: 2px;
    }}
    .chip {{
      font-size: 11px;
      padding: 3px 8px;
      border-radius: 4px;
      background: #1E293B;
      color: #94A3B8;
      white-space: nowrap;
      cursor: pointer;
      border: 1px solid #334155;
      transition: all 0.15s ease;
    }}
    .chip:hover {{
      background: #334155;
      color: #F8FAFC;
    }}
    .chip-tfr {{
      border-color: #F59E0B;
      color: #FBBF24;
    }}
    .chip-deliv {{
      border-color: #10B981;
      color: #34D399;
    }}
    .chip-replan {{
      border-color: #8B5CF6;
      color: #C084FC;
    }}
    /* Custom Map Markers */
    .plane-icon {{
      transition: transform 0.1s linear;
    }}
    .legend {{
      position: absolute;
      bottom: 16px;
      left: 16px;
      background: rgba(15, 23, 42, 0.9);
      border: 1px solid rgba(51, 65, 85, 0.8);
      border-radius: 8px;
      padding: 8px 12px;
      font-size: 11px;
      z-index: 1000;
      display: flex;
      flex-direction: column;
      gap: 5px;
    }}
    .legend-item {{
      display: flex;
      align-items: center;
      gap: 8px;
    }}
    .legend-line {{
      width: 18px;
      height: 3px;
      border-radius: 1px;
    }}
  </style>
</head>
<body>
  <header>
    <div class="brand">
      <div class="brand-icon">🚁</div>
      <div>
        <h1>Med-Drone Mission Control — SITL Telemetry</h1>
        <div style="font-size: 11px; color: #94A3B8;">Scenario: {scenario}</div>
      </div>
    </div>
    <div style="display: flex; gap: 8px;">
      <span class="badge badge-sitl">ArduPilot SITL EKF3</span>
      <span class="badge">Live Navigation Stream</span>
    </div>
  </header>

  <div class="main-container">
    <div id="map"></div>

    <div class="hud-panel">
      <div class="hud-title">
        <span>Autopilot Telemetry (EKF3)</span>
        <span id="hud-status" class="val-mode">STANDBY</span>
      </div>
      <div class="hud-grid">
        <div class="hud-metric">
          <span class="label">Flight Mode</span>
          <span id="hud-mode" class="val val-mode">--</span>
        </div>
        <div class="hud-metric">
          <span class="label">Armed State</span>
          <span id="hud-armed" class="val">--</span>
        </div>
        <div class="hud-metric">
          <span class="label">Alt Rel (AGL)</span>
          <span id="hud-alt" class="val">0.0 m</span>
        </div>
        <div class="hud-metric">
          <span class="label">Groundspeed</span>
          <span id="hud-speed" class="val">0.0 m/s</span>
        </div>
        <div class="hud-metric">
          <span class="label">Latitude</span>
          <span id="hud-lat" class="val">--</span>
        </div>
        <div class="hud-metric">
          <span class="label">Longitude</span>
          <span id="hud-lon" class="val">--</span>
        </div>
        <div class="hud-metric">
          <span class="label">Heading</span>
          <span id="hud-hdg" class="val">000°</span>
        </div>
        <div class="hud-metric">
          <span class="label">Active WP</span>
          <span id="hud-wp" class="val">#0</span>
        </div>
        <div class="hud-metric">
          <span class="label">Battery SoC</span>
          <span id="hud-bat" class="val">100%</span>
        </div>
        <div class="hud-metric">
          <span class="label">GNSS Fix / Sats</span>
          <span id="hud-gps" class="val">3D / 10</span>
        </div>
      </div>
    </div>

    <div class="legend">
      <div class="legend-item">
        <div class="legend-line" style="background: #38BDF8;"></div>
        <span>Measured Autopilot EKF Track (SITL)</span>
      </div>
      <div class="legend-item">
        <div class="legend-line" style="background: #94A3B8; border-top: 2px dashed #94A3B8;"></div>
        <span>Planned Mission Polyline</span>
      </div>
      <div class="legend-item">
        <div class="legend-line" style="background: #EF4444;"></div>
        <span>Restricted Airspace (No-Fly Zone)</span>
      </div>
      <div class="legend-item">
        <div class="legend-line" style="background: #F59E0B;"></div>
        <span>Injected TFR / Obstacle Exclusion</span>
      </div>
    </div>
  </div>

  <div class="controls-panel">
    <div class="scrub-row">
      <span id="time-display" class="time-badge">T+00:00</span>
      <input type="range" id="timeline" min="0" max="100" value="0" step="1">
      <span id="total-time-display" class="time-badge">T+00:00</span>
    </div>
    <div class="playback-row">
      <div class="btn-group">
        <button id="btn-play" class="btn active">▶ Play</button>
        <button id="btn-speed-1" class="btn active">1x</button>
        <button id="btn-speed-2" class="btn">2x</button>
        <button id="btn-speed-5" class="btn">5x</button>
        <button id="btn-speed-10" class="btn">10x</button>
      </div>
      <div class="event-chips" id="event-chips">
        <!-- Event chips populated dynamically -->
      </div>
    </div>
  </div>

  <script>
    const telemetry = {js_telemetry};
    const missionItems = {js_mission};
    const events = {js_events};
    const dynamicZones = {js_dynamic_zones};
    const homePos = {js_home};

    // Initialize Leaflet Map
    const map = L.map('map', {{
      center: homePos,
      zoom: 14,
      zoomControl: true,
    }});

    // Dark high-contrast base tiles
    L.tileLayer('https://{{s}}.basemaps.cartocdn.com/dark_all/{{z}}/{{x}}/{{y}}{{r}}.png', {{
      attribution: '&copy; OpenStreetMap contributors &copy; CARTO',
      subdomains: 'abcd',
      maxZoom: 19
    }}).addTo(map);

    // Depot marker
    const homeIcon = L.divIcon({{
      html: '<div style="background:#0F172A; border:2px solid #38BDF8; border-radius:50%; width:26px; height:26px; display:flex; align-items:center; justify-content:center; font-size:14px; box-shadow:0 0 10px rgba(56,189,248,0.5);">🏥</div>',
      className: '',
      iconSize: [26, 26],
      iconAnchor: [13, 13]
    }});
    L.marker(homePos, {{icon: homeIcon}}).addTo(map).bindPopup("<b>Depot / Launch Base</b><br>SMS Hospital Jaipur");

    // Draw planned mission path
    const planCoords = [];
    missionItems.forEach(it => {{
      if (it.lat !== 0.0 && it.role !== 'rtl') {{
        planCoords.push([it.lat, it.lon]);
        if (it.role === 'arrive' || it.role === 'hover_descend' || it.role === 'land') {{
          const stopIcon = L.divIcon({{
            html: `<div style="background:#1E293B; border:2px solid #34D399; color:#34D399; border-radius:50%; width:22px; height:22px; display:flex; align-items:center; justify-content:center; font-size:11px; font-weight:700;">${{it.stop || 'S'}}</div>`,
            className: '',
            iconSize: [22, 22],
            iconAnchor: [11, 11]
          }});
          L.marker([it.lat, it.lon], {{icon: stopIcon}}).addTo(map).bindPopup(`<b>Stop ${{it.stop}}</b><br>Role: ${{it.role}}`);
        }}
      }}
    }});
    if (planCoords.length > 0) {{
      planCoords.push(homePos);
      L.polyline(planCoords, {{
        color: '#64748B',
        weight: 2,
        dashArray: '6, 6',
        opacity: 0.7
      }}).addTo(map);
    }}

    // Draw dynamic/injected zones
    dynamicZones.forEach(z => {{
      L.polygon(z.polygon, {{
        color: '#F59E0B',
        weight: 2,
        fillColor: '#F59E0B',
        fillOpacity: 0.25,
      }}).addTo(map).bindPopup(`<b>${{z.name}}</b><br>TFR Injected during flight`);
    }});

    // Flight track polyline & animated drone marker
    const actualTrack = L.polyline([], {{
      color: '#38BDF8',
      weight: 3,
      opacity: 0.95
    }}).addTo(map);

    const droneIcon = L.divIcon({{
      html: '<div id="drone-marker" style="transform: rotate(0deg);"><svg width="32" height="32" viewBox="0 0 32 32"><circle cx="16" cy="16" r="14" fill="rgba(56,189,248,0.2)" stroke="#38BDF8" stroke-width="2"/><path d="M16 6 L22 22 L16 19 L10 22 Z" fill="#38BDF8"/></svg></div>',
      className: 'plane-icon',
      iconSize: [32, 32],
      iconAnchor: [16, 16]
    }});
    const droneMarker = L.marker(homePos, {{icon: droneIcon}}).addTo(map);

    // Playback state
    let isPlaying = true;
    let playbackSpeed = 1.0;
    let currentIndex = 0;
    const totalSamples = telemetry.length;

    const timeline = document.getElementById('timeline');
    timeline.max = Math.max(0, totalSamples - 1);

    const hudMode = document.getElementById('hud-mode');
    const hudArmed = document.getElementById('hud-armed');
    const hudAlt = document.getElementById('hud-alt');
    const hudSpeed = document.getElementById('hud-speed');
    const hudLat = document.getElementById('hud-lat');
    const hudLon = document.getElementById('hud-lon');
    const hudHdg = document.getElementById('hud-hdg');
    const hudWp = document.getElementById('hud-wp');
    const hudBat = document.getElementById('hud-bat');
    const hudGps = document.getElementById('hud-gps');
    const hudStatus = document.getElementById('hud-status');
    const timeDisplay = document.getElementById('time-display');
    const totalTimeDisplay = document.getElementById('total-time-display');

    if (totalSamples > 0) {{
      const totalSec = Math.round(telemetry[totalSamples - 1].t - telemetry[0].t);
      const tm = Math.floor(totalSec / 60);
      const ts = totalSec % 60;
      totalTimeDisplay.innerText = `T+${{String(tm).padStart(2,'0')}}:${{String(ts).padStart(2,'0')}}`;
    }}

    function updateFrame(index) {{
      if (totalSamples === 0) return;
      currentIndex = Math.min(Math.max(0, index), totalSamples - 1);
      timeline.value = currentIndex;
      const s = telemetry[currentIndex];

      // Update HUD
      hudMode.innerText = s.flight_mode;
      hudArmed.innerText = s.armed ? 'ARMED' : 'DISARMED';
      hudArmed.className = s.armed ? 'val val-mode' : 'val val-warn';
      hudAlt.innerText = `${{s.alt_rel_m.toFixed(1)}} m`;
      hudSpeed.innerText = `${{s.groundspeed_ms.toFixed(1)}} m/s`;
      hudLat.innerText = s.lat.toFixed(5);
      hudLon.innerText = s.lon.toFixed(5);
      hudHdg.innerText = `${{Math.round(s.heading_deg || 0)}}°`;
      hudWp.innerText = s.mission_current !== null ? `#${{s.mission_current}}` : '--';
      if (s.battery && s.battery.remaining_pct !== undefined) {{
        hudBat.innerText = `${{s.battery.remaining_pct}}%`;
      }}
      if (s.gnss) {{
        hudGps.innerText = `${{s.gnss.fix_type >= 3 ? '3D' : 'No Fix'}} / ${{s.gnss.satellites || 0}}`;
      }}
      hudStatus.innerText = s.flight_mode;

      const sec = Math.round(s.t - telemetry[0].t);
      const m = Math.floor(sec / 60);
      const sc = sec % 60;
      timeDisplay.innerText = `T+${{String(m).padStart(2,'0')}}:${{String(sc).padStart(2,'0')}}`;

      // Update Drone Marker & Map
      const pos = [s.lat, s.lon];
      droneMarker.setLatLng(pos);
      const rotElem = document.getElementById('drone-marker');
      if (rotElem) {{
        rotElem.style.transform = `rotate(${{s.heading_deg || 0}}deg)`;
      }}

      // Update Flight Track line up to currentIndex
      const history = telemetry.slice(0, currentIndex + 1).map(pt => [pt.lat, pt.lon]);
      actualTrack.setLatLngs(history);
    }}

    // Populate Event Chips
    const chipsContainer = document.getElementById('event-chips');
    events.forEach(ev => {{
      const name = ev.event;
      if (name === 'takeoff' || name === 'delivery_complete' || name === 'airspace_conflict' || name === 'mission_replanned' || name === 'mission_finished') {{
        const chip = document.createElement('div');
        chip.className = 'chip';
        if (name === 'delivery_complete') {{
          chip.className += ' chip-deliv';
          chip.innerText = `✓ Delivered #${{ev.stop_id}}`;
        }} else if (name === 'airspace_conflict') {{
          chip.className += ' chip-tfr';
          chip.innerText = `⚠ Airspace Conflict`;
        }} else if (name === 'mission_replanned') {{
          chip.className += ' chip-replan';
          chip.innerText = `🔄 Mission Replanned v${{ev.version || 2}}`;
        }} else if (name === 'mission_finished') {{
          chip.innerText = `🏁 Mission Completed`;
        }} else {{
          chip.innerText = name;
        }}

        chip.addEventListener('click', () => {{
          // Find sample closest to ev.t
          const idx = telemetry.findIndex(s => s.t >= ev.t);
          if (idx !== -1) {{
            updateFrame(idx);
          }}
        }});
        chipsContainer.appendChild(chip);
      }}
    }});

    // Timeline input
    timeline.addEventListener('input', (e) => {{
      isPlaying = false;
      document.getElementById('btn-play').innerText = '▶ Play';
      updateFrame(parseInt(e.target.value, 10));
    }});

    // Play/Pause button
    const btnPlay = document.getElementById('btn-play');
    btnPlay.addEventListener('click', () => {{
      isPlaying = !isPlaying;
      btnPlay.innerText = isPlaying ? '⏸ Pause' : '▶ Play';
    }});

    // Speed buttons
    const speeds = [1, 2, 5, 10];
    speeds.forEach(spd => {{
      const btn = document.getElementById(`btn-speed-${{spd}}`);
      if (btn) {{
        btn.addEventListener('click', () => {{
          playbackSpeed = spd;
          speeds.forEach(s => {{
            const b = document.getElementById(`btn-speed-${{s}}`);
            if (b) b.classList.remove('active');
          }});
          btn.classList.add('active');
        }});
      }}
    }});

    // Animation Loop
    let lastAnimTime = performance.now();
    function animate(now) {{
      const dt = (now - lastAnimTime) / 1000.0;
      lastAnimTime = now;

      if (isPlaying && totalSamples > 0) {{
        // telemetry sample rate is ~1 Hz; step proportional to dt * playbackSpeed
        currentIndex += Math.max(1, Math.round(dt * playbackSpeed * 10));
        if (currentIndex >= totalSamples) {{
          currentIndex = totalSamples - 1;
          isPlaying = false;
          btnPlay.innerText = '▶ Play';
        }}
        updateFrame(currentIndex);
      }}
      requestAnimationFrame(animate);
    }}
    requestAnimationFrame(animate);

    // Initial frame
    if (totalSamples > 0) {{
      updateFrame(0);
      map.fitBounds(actualTrack.getBounds().pad(0.15));
    }}
  </script>
</body>
</html>
"""

    os.makedirs(os.path.dirname(os.path.abspath(output_html_path)), exist_ok=True)
    with open(output_html_path, "w", encoding="utf-8") as f:
        f.write(html_content)
    print(f"[visualize_sitl] Wrote interactive SITL replay to: {output_html_path}")


def main():
    parser = argparse.ArgumentParser(description="Render interactive SITL flight replay HTML")
    parser.add_argument("log_path", nargs="?", default=None, help="Path to sitl_run_*.jsonl (default: latest in output/)")
    parser.add_argument("-o", "--output", default="output/sitl_flight_replay.html", help="Output HTML file path")
    args = parser.parse_args()

    log_path = args.log_path or find_latest_sitl_log()
    if not log_path or not os.path.isfile(log_path):
        print(f"[visualize_sitl] Error: No SITL run log found at {log_path or 'output/sitl_run_*.jsonl'}")
        print("Run `python3 sitl_mission_demo.py` first to generate a SITL telemetry log.")
        return 1

    print(f"[visualize_sitl] Parsing SITL log: {log_path}")
    data = parse_sitl_log(log_path)
    generate_html_dashboard(data, args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
