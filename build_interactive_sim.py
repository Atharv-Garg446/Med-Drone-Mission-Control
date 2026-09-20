"""
build_interactive_sim.py
========================
Generates a standalone interactive HTML simulation from a FleetSimulator's
recorded telemetry data.

Features:
 - Real Leaflet map with OpenStreetMap tiles (works via HTTP server)
 - Smooth animated drone markers moving along recorded paths
 - Play/pause/speed controls (1x, 5x, 10x, 50x)
 - Timeline scrubber
 - Live event log panel with color-coded entries
 - Delivery progress tracker
 - Colored drone trails showing paths taken
 - No-fly zones (static + dynamic TFRs appearing mid-mission)
 - Emergency deliveries popping up with pulse animation
 - Mission statistics dashboard

Usage:
    from build_interactive_sim import build_interactive_html
    sim = FleetSimulator(city, events=events)
    report = sim.run()
    build_interactive_html(sim, "output/interactive_sim.html")
"""

import json
import os


def build_interactive_html(sim, output_path="output/interactive_sim.html"):
    """Build a standalone interactive HTML simulation viewer."""

    city = sim.city
    depot = city.depot

    # Delivery locations
    locations_data = []
    for loc in sim.locations:
        locations_data.append({
            "id": loc.id, "name": loc.name,
            "lat": loc.lat, "lon": loc.lon,
            "demand_kg": loc.demand_kg, "urgency": loc.urgency,
            "is_depot": loc.id == 0,
        })
    for loc in sim.emergency_locations:
        locations_data.append({
            "id": loc.id, "name": loc.name,
            "lat": loc.lat, "lon": loc.lon,
            "demand_kg": loc.demand_kg, "urgency": loc.urgency,
            "is_depot": False, "is_emergency": True,
        })

    # Position frames (sample every 10 ticks to keep file size manageable)
    frames = {}
    for pos in sim.position_history:
        tick = pos["tick"]
        if tick not in frames:
            frames[tick] = []
        frames[tick].append({
            "d": pos["drone_id"],
            "la": round(pos["lat"], 6),
            "lo": round(pos["lon"], 6),
        })
    sorted_frames = sorted(frames.items(), key=lambda x: x[0])
    frames_list = [{"t": tick, "d": drones} for tick, drones in sorted_frames]

    # Events, deliveries, NFZs
    events_data = sim.event_history
    deliveries_data = sim.delivery_history
    static_nfz = [list(zone) for zone in city.no_fly_zones]
    dynamic_nfz = [{"tick": n["tick"], "zone": list(n["zone"]),
                     "reason": n.get("reason", "TFR")} for n in sim.nfz_history]

    report_data = {
        "city_name": city.name,
        "scenario": city.scenario_description[:200] if city.scenario_description else city.name,
        "total_deliveries": len(sim.all_delivery_ids),
        "completed": len(sim.delivered_ids),
        "total_drones": len(sim.drones),
        "replans": sim.replans,
        "replan_ms": sim.total_replan_ms,
        "max_tick": sim.tick,
    }

    # Log lines
    log_lines = sim.log if hasattr(sim, 'log') else []

    html = _generate_html(
        depot_lat=depot.lat, depot_lon=depot.lon,
        locations=json.dumps(locations_data, separators=(',', ':')),
        frames=json.dumps(frames_list, separators=(',', ':')),
        events=json.dumps(events_data, separators=(',', ':')),
        deliveries=json.dumps(deliveries_data, separators=(',', ':')),
        static_nfz=json.dumps(static_nfz, separators=(',', ':')),
        dynamic_nfz=json.dumps(dynamic_nfz, separators=(',', ':')),
        report=json.dumps(report_data, separators=(',', ':')),
        logs=json.dumps(log_lines, separators=(',', ':')),
    )

    os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else ".", exist_ok=True)
    with open(output_path, "w") as f:
        f.write(html)
    return output_path


def _generate_html(depot_lat, depot_lon, locations, frames, events,
                   deliveries, static_nfz, dynamic_nfz, report, logs):
    # Use raw strings and explicit format to avoid f-string/CSS brace conflicts
    return """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Drone Medical Router — Interactive Simulation</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
      integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY="
      crossorigin=""/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
        integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo="
        crossorigin=""></script>
<style>
*{margin:0;padding:0;box-sizing:border-box}
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500&display=swap');
body{font-family:'Inter',system-ui,sans-serif;background:#0d1117;color:#c9d1d9;overflow:hidden;height:100vh}
#map{position:absolute;top:0;left:0;right:340px;bottom:0;z-index:1}
.leaflet-control-attribution{font-size:9px!important;background:rgba(13,17,23,0.7)!important;color:#8b949e!important}
.leaflet-control-attribution a{color:#58a6ff!important}

#sidebar{position:absolute;top:0;right:0;width:340px;bottom:0;
  background:linear-gradient(180deg,#161b22 0%,#0d1117 100%);
  border-left:1px solid #30363d;display:flex;flex-direction:column;z-index:2;overflow:hidden}

.hdr{padding:16px 18px;background:linear-gradient(135deg,#1f2937,#111827);border-bottom:1px solid #30363d}
.hdr h1{font-size:15px;font-weight:700;background:linear-gradient(90deg,#58a6ff,#3fb950);-webkit-background-clip:text;-webkit-text-fill-color:transparent;letter-spacing:.3px}
.hdr .sc{font-size:11px;color:#8b949e;margin-top:4px;line-height:1.3}

.ctrl{padding:14px 18px;border-bottom:1px solid #30363d}
.timer{font-size:26px;font-weight:700;color:#f0f6fc;font-family:'JetBrains Mono',monospace;letter-spacing:1px}
.ctrl-row{display:flex;gap:6px;margin-top:10px}
.btn{padding:7px 14px;border:1px solid #30363d;background:#21262d;color:#c9d1d9;border-radius:8px;cursor:pointer;font-size:12px;font-weight:500;transition:all .15s;font-family:'Inter',sans-serif}
.btn:hover{background:#30363d;color:#f0f6fc;border-color:#58a6ff}
.btn.active{background:#1f6feb;border-color:#58a6ff;color:#fff}
.btn.go{background:#238636;border-color:#3fb950}
.btn.go:hover{background:#2ea043}

.pbar{height:6px;background:#21262d;border-radius:3px;margin-top:12px;cursor:pointer;position:relative}
.pfill{height:100%;background:linear-gradient(90deg,#58a6ff,#3fb950);border-radius:3px;transition:width .05s}

.stats{padding:12px 18px;border-bottom:1px solid #30363d}
.sg{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.si{background:#161b22;padding:10px 12px;border-radius:8px;border:1px solid #21262d}
.sl{font-size:10px;color:#8b949e;text-transform:uppercase;letter-spacing:.8px;font-weight:600}
.sv{font-size:20px;font-weight:700;color:#58a6ff;margin-top:3px;font-family:'JetBrains Mono',monospace}
.sv.g{color:#3fb950}.sv.w{color:#f0883e}.sv.r{color:#f85149}

.elog{flex:1;overflow-y:auto;padding:10px 18px}
.elog .lbl{font-size:10px;color:#8b949e;text-transform:uppercase;letter-spacing:.8px;font-weight:600;position:sticky;top:0;background:#0d1117;padding:4px 0;z-index:1}
.le{font-size:11px;padding:5px 0;border-bottom:1px solid #161b22;line-height:1.4;font-family:'JetBrains Mono',monospace}
.le .tm{color:#58a6ff;font-weight:600;margin-right:6px}
.le.ev{color:#f0883e}.le.dl{color:#3fb950}.le.rp{color:#bc8cff}

.leg{padding:12px 18px;border-top:1px solid #30363d;display:flex;flex-wrap:wrap;gap:8px}
.li{display:flex;align-items:center;gap:5px;font-size:10px;color:#8b949e}
.ld{width:10px;height:10px;border-radius:50%}
.lr{width:14px;height:10px;border-radius:2px}
</style>
</head>
<body>
<div id="map"></div>
<div id="sidebar">
  <div class="hdr">
    <h1>🚁 DRONE MEDICAL ROUTER</h1>
    <div class="sc" id="sc"></div>
  </div>
  <div class="ctrl">
    <div style="display:flex;justify-content:space-between;align-items:center">
      <div class="timer" id="timer">T+00:00</div>
      <div style="display:flex;gap:5px">
        <button class="btn go" id="pbtn" onclick="toggle()">▶ Play</button>
        <button class="btn" onclick="reset()">⟲ Reset</button>
      </div>
    </div>
    <div class="ctrl-row">
      <button class="btn active" id="s1" onclick="spd(1,this)">1×</button>
      <button class="btn" id="s5" onclick="spd(5,this)">5×</button>
      <button class="btn" id="s10" onclick="spd(10,this)">10×</button>
      <button class="btn" id="s50" onclick="spd(50,this)">50×</button>
    </div>
    <div class="pbar" id="pbar" onclick="seek(event)">
      <div class="pfill" id="pfill" style="width:0%"></div>
    </div>
  </div>
  <div class="stats">
    <div class="sg">
      <div class="si"><div class="sl">Deliveries</div><div class="sv g" id="xdel">0/0</div></div>
      <div class="si"><div class="sl">Active Drones</div><div class="sv" id="xdr">0</div></div>
      <div class="si"><div class="sl">Replans</div><div class="sv w" id="xrp">0</div></div>
      <div class="si"><div class="sl">Events</div><div class="sv r" id="xev">0</div></div>
    </div>
  </div>
  <div class="elog" id="elog"><div class="lbl">📋 Mission Log</div></div>
  <div class="leg">
    <div class="li"><div class="ld" style="background:#000;border:2px solid #fff"></div>Depot</div>
    <div class="li"><div class="ld" style="background:#58a6ff"></div>Delivery</div>
    <div class="li"><div class="ld" style="background:#f85149"></div>Emergency</div>
    <div class="li"><div class="ld" style="background:#3fb950"></div>Delivered ✓</div>
    <div class="li"><div class="lr" style="background:rgba(248,81,73,.25);border:1px solid #f85149"></div>No-Fly</div>
    <div class="li"><div class="lr" style="background:rgba(240,136,62,.25);border:1px solid #f0883e"></div>Pop-up TFR</div>
  </div>
</div>

<script>
const LOC=""" + locations + """;
const FR=""" + frames + """;
const EV=""" + events + """;
const DL=""" + deliveries + """;
const SNF=""" + static_nfz + """;
const DNF=""" + dynamic_nfz + """;
const RP=""" + report + """;
const LG=""" + logs + """;
const DC=['#58a6ff','#f85149','#3fb950','#bc8cff','#f0883e','#39d2c0','#e3507a','#79c0ff','#ff7b72','#56d364'];

let map,on=false,sp=1,fi=0,dm={},trails={},dlSet=new Set(),evSet=new Set(),nfSet=new Set();
let nDel=0,nRp=0,nEv=0,lt=0,dynLayers=[];

function init(){
  document.getElementById('sc').textContent=RP.scenario||RP.city_name;
  map=L.map('map',{center:[""" + str(depot_lat) + "," + str(depot_lon) + """],zoom:12,zoomControl:true});

  // Esri World Dark Gray (Zero API keys, zero watermarks, high contrast dark theme)
  const darkBase=L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/Canvas/World_Dark_Gray_Base/MapServer/tile/{z}/{y}/{x}',{
    attribution:'Tiles &copy; Esri &mdash; Esri, DeLorme, NAVTEQ',
    maxZoom:16
  }).addTo(map);

  const darkLabels=L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/Canvas/World_Dark_Gray_Reference/MapServer/tile/{z}/{y}/{x}',{
    attribution:'',
    maxZoom:16
  }).addTo(map);

  const satellite=L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',{
    attribution:'Tiles &copy; Esri',
    maxZoom:18
  });

  L.control.layers({
    "🌙 Dark Mode": darkBase,
    "🛰️ Satellite": satellite
  },{
    "Labels & Roads": darkLabels
  },{position:'topleft'}).addTo(map);

  // Static no-fly zones
  SNF.forEach(z=>{
    L.polygon(z,{color:'#f85149',weight:2,fillColor:'#f85149',fillOpacity:.15,dashArray:'5,5'})
     .addTo(map).bindTooltip('⛔ No-Fly Zone',{sticky:true});
  });

  // Depot
  L.circleMarker([""" + str(depot_lat) + "," + str(depot_lon) + """],{
    radius:12,color:'#f0f6fc',weight:3,fillColor:'#0d1117',fillOpacity:1
  }).addTo(map).bindTooltip('🏥 '+LOC[0].name,{permanent:true,direction:'right',offset:[15,0],
    className:'depot-tip'});

  // Delivery locations
  LOC.forEach(l=>{
    if(l.is_depot)return;
    if(l.is_emergency)return; // emergencies shown dynamically
    const c=l.urgency==='critical'?'#f85149':l.urgency==='urgent'?'#f0883e':'#58a6ff';
    L.circleMarker([l.lat,l.lon],{radius:7,color:c,weight:2,fillColor:c,fillOpacity:.6})
     .addTo(map).bindTooltip(l.name+'<br>'+l.demand_kg+'kg • '+l.urgency,{sticky:true});
  });

  document.getElementById('xdel').textContent='0/'+RP.total_deliveries;
  document.getElementById('xdr').textContent=RP.total_drones;
  showLog(0);
}

function toggle(){
  on=!on;
  document.getElementById('pbtn').textContent=on?'⏸ Pause':'▶ Play';
  document.getElementById('pbtn').classList.toggle('go',!on);
  if(on){lt=performance.now();anim();}
}

function spd(s,el){
  sp=s;
  document.querySelectorAll('.ctrl-row .btn').forEach(b=>b.classList.remove('active'));
  el.classList.add('active');
}

function reset(){
  on=false;fi=0;nDel=0;nRp=0;nEv=0;
  dlSet.clear();evSet.clear();nfSet.clear();
  document.getElementById('pbtn').textContent='▶ Play';
  document.getElementById('pbtn').classList.add('go');
  Object.values(dm).forEach(m=>map.removeLayer(m));
  Object.values(trails).forEach(t=>map.removeLayer(t));
  dynLayers.forEach(l=>map.removeLayer(l));
  dm={};trails={};dynLayers=[];
  upd(0);stats();
}

function seek(e){
  const bar=document.getElementById('pbar');
  const pct=Math.max(0,Math.min(1,e.offsetX/bar.offsetWidth));
  fi=Math.floor(pct*(FR.length-1));
  upd(FR[fi]?FR[fi].t:0);
}

function anim(){
  if(!on||fi>=FR.length){
    if(fi>=FR.length){on=false;document.getElementById('pbtn').textContent='▶ Play';document.getElementById('pbtn').classList.add('go');}
    return;
  }
  const now=performance.now();
  const dt=(now-lt)/1000;
  lt=now;
  let adv=Math.max(1,Math.floor(dt*sp*8));
  fi=Math.min(fi+adv,FR.length-1);
  upd(FR[fi].t);
  requestAnimationFrame(anim);
}

function upd(tick){
  const f=FR[fi];if(!f)return;

  // Time display
  const m=Math.floor(tick/60),s=tick%60;
  document.getElementById('timer').textContent='T+'+String(m).padStart(2,'0')+':'+String(s).padStart(2,'0');
  const mx=FR[FR.length-1]?FR[FR.length-1].t:1;
  document.getElementById('pfill').style.width=(tick/mx*100)+'%';

  // Drone positions + trails
  f.d.forEach(p=>{
    const col=DC[(p.d-1)%DC.length];
    if(dm[p.d]){
      dm[p.d].setLatLng([p.la,p.lo]);
      // Add to trail
      if(trails[p.d]){
        const ll=trails[p.d].getLatLngs();
        const last=ll[ll.length-1];
        if(!last||last.lat!==p.la||last.lng!==p.lo){
          trails[p.d].addLatLng([p.la,p.lo]);
        }
      }
    } else {
      // Create drone marker
      dm[p.d]=L.circleMarker([p.la,p.lo],{
        radius:9,color:'#f0f6fc',weight:2,fillColor:col,fillOpacity:.95,
      }).addTo(map);
      dm[p.d].bindTooltip('🚁 Drone '+p.d,{permanent:true,direction:'top',offset:[0,-12],
        className:'drone-tip'});
      // Create trail
      trails[p.d]=L.polyline([[p.la,p.lo]],{
        color:col,weight:2,opacity:.4,dashArray:'4,6'
      }).addTo(map);
    }
  });

  // Show events
  EV.forEach((ev,i)=>{
    if(ev.tick<=tick&&!evSet.has(i)){
      evSet.add(i);nEv++;
      if(ev.type==='emergency'&&ev.lat&&ev.lon){
        const m=L.circleMarker([ev.lat,ev.lon],{
          radius:10,color:'#f85149',weight:3,fillColor:'#f85149',fillOpacity:.5
        }).addTo(map).bindTooltip('🚨 '+(ev.name||'Emergency'),{sticky:true});
        dynLayers.push(m);
      }
    }
  });

  // Show dynamic TFRs
  DNF.forEach((nfz,i)=>{
    if(nfz.tick<=tick&&!nfSet.has(i)){
      nfSet.add(i);
      const p=L.polygon(nfz.zone,{
        color:'#f0883e',weight:3,fillColor:'#f0883e',fillOpacity:.2,dashArray:'8,4'
      }).addTo(map).bindTooltip('⚡ '+nfz.reason,{sticky:true});
      dynLayers.push(p);
    }
  });

  // Show deliveries
  DL.forEach((dl,i)=>{
    if(dl.tick<=tick&&!dlSet.has(i)){
      dlSet.add(i);nDel++;
      const m=L.circleMarker([dl.lat,dl.lon],{
        radius:11,color:'#3fb950',weight:3,fillColor:'#3fb950',fillOpacity:.6,
      }).addTo(map).bindTooltip('✅ '+dl.name,{sticky:true});
      dynLayers.push(m);
    }
  });

  // Count replans
  nRp=0;
  LG.forEach(line=>{
    const mt=line.match(/T\\+(\\d+):(\\d+)/);
    if(mt){
      const lt2=parseInt(mt[1])*60+parseInt(mt[2]);
      if(lt2<=tick&&line.includes('REPLANNING'))nRp++;
    }
  });

  stats();
  showLog(tick);
}

function stats(){
  document.getElementById('xdel').textContent=nDel+'/'+RP.total_deliveries;
  document.getElementById('xdr').textContent=Object.keys(dm).length;
  document.getElementById('xrp').textContent=nRp;
  document.getElementById('xev').textContent=nEv;
}

function showLog(tick){
  const el=document.getElementById('elog');
  let h='<div class="lbl">📋 Mission Log</div>';
  LG.forEach(line=>{
    const mt=line.match(/T\\+(\\d+):(\\d+)/);
    if(mt&&parseInt(mt[1])*60+parseInt(mt[2])>tick)return;
    let c='le';
    if(line.includes('EVENT')||line.includes('⚡')||line.includes('🚨')||line.includes('💥'))c+=' ev';
    else if(line.includes('📦')||line.includes('🏠'))c+=' dl';
    else if(line.includes('REPLAN')||line.includes('✅ Replanned'))c+=' rp';
    h+='<div class="'+c+'">'+line+'</div>';
  });
  el.innerHTML=h;
  el.scrollTop=el.scrollHeight;
}

init();
</script>
</body>
</html>"""


if __name__ == "__main__":
    import random
    from config import JAIPUR_DISASTER
    from simulate_fleet import FleetSimulator, generate_random_events

    city = JAIPUR_DISASTER
    rng = random.Random(42)
    events = generate_random_events(city, 3, 3600, rng)
    sim = FleetSimulator(city, events=events, speed_multiplier=100)
    sim.run(max_ticks=3600)

    build_interactive_html(sim, "output/interactive_sim.html")
    build_interactive_html(sim, "docs/index.html")
    print("Generated interactive simulation at output/interactive_sim.html and docs/index.html")

