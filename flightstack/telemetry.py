"""
flightstack/telemetry.py
========================
Autopilot telemetry state, built ONLY from MAVLink messages received from the
flight controller (ArduPilot SITL or real hardware).

Three kinds of state exist in this project and must not be confused:

  * Simulated ground truth  -- what the physics simulator knows. The planner
                               never reads it.
  * Autopilot telemetry     -- the autopilot's own *estimate* (EKF position,
                               velocity, battery monitor reading, GNSS fix).
                               This module stores it.
  * Planner state           -- which stops are delivered, which route is
                               active (see flightstack/mission_runner.py).

Every field group carries the local monotonic receive time so callers can
reject stale data explicitly instead of silently acting on an old position.
This module has no pymavlink dependency: it accepts any object that exposes
``get_type()`` and MAVLink field attributes, which keeps it unit-testable.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional, Tuple

# ---------------------------------------------------------------------------
# MAVLink constants used here (copied from the MAVLink common/ardupilotmega
# dialect so this module does not need pymavlink at import time).
# ---------------------------------------------------------------------------
MAV_MODE_FLAG_SAFETY_ARMED = 128
GPS_FIX_TYPE_3D = 3

# EKF_STATUS_REPORT.flags bits (ardupilotmega EKF_STATUS_FLAGS)
EKF_ATTITUDE = 1
EKF_VELOCITY_HORIZ = 2
EKF_POS_HORIZ_ABS = 16
EKF_CONST_POS_MODE = 128
EKF_GPS_GLITCHING = 32768

# ArduCopter custom_mode numbers -> names (ArduCopter/mode.h)
COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
    13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE",
    18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
    22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG", 25: "SYSTEMID",
    26: "AUTOROTATE", 27: "AUTO_RTL",
}

DEFAULT_MAX_AGE_S = 2.0   # position/attitude older than this is "stale"


class StaleTelemetryError(RuntimeError):
    """Raised when a caller requires fresh telemetry and it is not available."""


@dataclass
class Stamped:
    """A value plus the local monotonic time (seconds) it was received."""
    value: object = None
    rx_time: Optional[float] = None

    def age(self, now: float) -> float:
        return math.inf if self.rx_time is None else now - self.rx_time


@dataclass
class PositionEstimate:
    lat: float
    lon: float
    alt_amsl_m: float
    alt_rel_m: float
    vn_ms: float
    ve_ms: float
    vd_ms: float
    heading_deg: Optional[float]
    autopilot_time_ms: int

    @property
    def groundspeed_ms(self) -> float:
        return math.hypot(self.vn_ms, self.ve_ms)


@dataclass
class GnssHealth:
    fix_type: int
    satellites: int
    eph_m: Optional[float]       # horizontal dilution * 1 m, None if unknown

    @property
    def has_3d_fix(self) -> bool:
        return self.fix_type >= GPS_FIX_TYPE_3D


@dataclass
class BatteryReading:
    voltage_v: Optional[float]
    current_a: Optional[float]
    remaining_pct: Optional[int]     # autopilot battery-monitor estimate
    consumed_mah: Optional[float]


@dataclass
class EkfHealth:
    flags: int

    @property
    def position_ok(self) -> bool:
        need = EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_POS_HORIZ_ABS
        return (self.flags & need) == need and not (self.flags & EKF_CONST_POS_MODE) \
            and not (self.flags & EKF_GPS_GLITCHING)


@dataclass
class TelemetryState:
    """Latest autopilot-reported state with per-group receive timestamps."""
    clock: Callable[[], float] = field(default=time.monotonic, repr=False)
    position: Stamped = field(default_factory=Stamped)
    gnss: Stamped = field(default_factory=Stamped)
    battery: Stamped = field(default_factory=Stamped)
    ekf: Stamped = field(default_factory=Stamped)
    heartbeat: Stamped = field(default_factory=Stamped)   # value = (mode, armed)
    mission_current: Stamped = field(default_factory=Stamped)
    last_reached_seq: Stamped = field(default_factory=Stamped)
    sim_ground_truth: Stamped = field(default_factory=Stamped)
    oa_db_items: list = field(default_factory=list)
    proximity_sectors_cm: Dict[int, int] = field(default_factory=dict)
    proximity_sector_rx: Dict[int, float] = field(default_factory=dict)
    rejected: Dict[str, int] = field(default_factory=dict)
    status_texts: list = field(default_factory=list)

    # ------------------------------------------------------------------
    # Ingest
    # ------------------------------------------------------------------
    def _reject(self, reason: str) -> None:
        self.rejected[reason] = self.rejected.get(reason, 0) + 1

    def update(self, msg) -> None:
        """Consume one MAVLink message. Unknown types are ignored."""
        mtype = msg.get_type()
        now = self.clock()
        handler = getattr(self, "_on_" + mtype, None)
        if handler is not None:
            handler(msg, now)

    def _on_HEARTBEAT(self, msg, now):
        # Ignore heartbeats from GCS/companion components (type 6 = GCS).
        if getattr(msg, "type", None) == 6:
            return
        mode = COPTER_MODES.get(msg.custom_mode, "MODE_%d" % msg.custom_mode)
        armed = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
        self.heartbeat = Stamped((mode, armed), now)

    def _on_GLOBAL_POSITION_INT(self, msg, now):
        lat, lon = msg.lat / 1e7, msg.lon / 1e7
        # Autopilots send 0,0 before the EKF has an origin -> not a position.
        if not (-90 <= lat <= 90 and -180 <= lon <= 180) or (msg.lat == 0 and msg.lon == 0):
            self._reject("invalid_position")
            return
        hdg = None if msg.hdg == 65535 else msg.hdg / 100.0
        self.position = Stamped(PositionEstimate(
            lat=lat, lon=lon,
            alt_amsl_m=msg.alt / 1000.0, alt_rel_m=msg.relative_alt / 1000.0,
            vn_ms=msg.vx / 100.0, ve_ms=msg.vy / 100.0, vd_ms=msg.vz / 100.0,
            heading_deg=hdg, autopilot_time_ms=msg.time_boot_ms,
        ), now)

    def _on_GPS_RAW_INT(self, msg, now):
        eph = None if msg.eph == 65535 else msg.eph / 100.0
        sats = msg.satellites_visible if msg.satellites_visible != 255 else 0
        self.gnss = Stamped(GnssHealth(msg.fix_type, sats, eph), now)

    def _on_SYS_STATUS(self, msg, now):
        v = None if msg.voltage_battery in (0, 65535) else msg.voltage_battery / 1000.0
        c = None if msg.current_battery < 0 else msg.current_battery / 100.0
        r = None if msg.battery_remaining < 0 else int(msg.battery_remaining)
        prev = self.battery.value
        consumed = prev.consumed_mah if isinstance(prev, BatteryReading) else None
        self.battery = Stamped(BatteryReading(v, c, r, consumed), now)

    def _on_BATTERY_STATUS(self, msg, now):
        prev = self.battery.value
        consumed = None if msg.current_consumed < 0 else float(msg.current_consumed)
        if isinstance(prev, BatteryReading):
            self.battery = Stamped(BatteryReading(prev.voltage_v, prev.current_a,
                                                  prev.remaining_pct, consumed), now)

    def _on_EKF_STATUS_REPORT(self, msg, now):
        self.ekf = Stamped(EkfHealth(msg.flags), now)

    def _on_MISSION_CURRENT(self, msg, now):
        self.mission_current = Stamped(int(msg.seq), now)

    def _on_MISSION_ITEM_REACHED(self, msg, now):
        self.last_reached_seq = Stamped(int(msg.seq), now)

    def _on_STATUSTEXT(self, msg, now):
        text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
        self.status_texts.append((now, int(msg.severity), text.rstrip("\x00")))
        del self.status_texts[:-200]

    def _on_ADSB_VEHICLE(self, msg, now):
        """ArduPilot AP_OADatabase streams internal OA database points via ADSB_VEHICLE with callsign 'OA_DB'."""
        raw_cs = getattr(msg, "callsign", "")
        cs = raw_cs if isinstance(raw_cs, str) else raw_cs.decode(errors="ignore")
        cs = cs.rstrip("\x00").strip()
        if cs.startswith("OA_DB"):
            lat = getattr(msg, "lat", 0) / 1e7
            lon = getattr(msg, "lon", 0) / 1e7
            radius_m = getattr(msg, "squawk", 0) / 100.0
            icao = int(getattr(msg, "ICAO_address", 0))
            if radius_m > 0:
                self.oa_db_items.append({
                    "rx_time": now,
                    "idx": icao,
                    "lat": lat,
                    "lon": lon,
                    "alt_m": getattr(msg, "altitude", 0) / 1000.0,
                    "radius_m": radius_m,
                })
                del self.oa_db_items[:-50]

    def _on_DISTANCE_SENSOR(self, msg, now):
        """ArduPilot AP_Proximity echoes horizontal sector distances (orientations 0..7) via DISTANCE_SENSOR."""
        orient = int(getattr(msg, "orientation", 0))
        if not (0 <= orient <= 7):
            return
        cur_cm = int(getattr(msg, "current_distance", 65535))
        max_cm = int(getattr(msg, "max_distance", 5000))
        effective_max_cm = min(max_cm, 5000)
        if 0 < cur_cm < effective_max_cm:
            self.proximity_sectors_cm[orient] = cur_cm
            self.proximity_sector_rx[orient] = now
        elif orient in self.proximity_sectors_cm:
            del self.proximity_sectors_cm[orient]
            self.proximity_sector_rx.pop(orient, None)
        for k in list(self.proximity_sectors_cm.keys()):
            if now - self.proximity_sector_rx.get(k, now) > 1.5:
                del self.proximity_sectors_cm[k]
                self.proximity_sector_rx.pop(k, None)

    def _on_SIMSTATE(self, msg, now):
        """Simulator ground truth state (pure physics, non-EKF)."""
        lat_raw = getattr(msg, "lat", 0.0)
        lon_raw = getattr(msg, "lng", 0.0)
        lat = lat_raw / 1e7 if abs(lat_raw) > 1000 else lat_raw
        lon = lon_raw / 1e7 if abs(lon_raw) > 1000 else lon_raw
        self.sim_ground_truth = Stamped({
            "lat": lat, "lon": lon,
            "roll_deg": round(math.degrees(getattr(msg, "roll", 0.0)), 2),
            "pitch_deg": round(math.degrees(getattr(msg, "pitch", 0.0)), 2),
            "yaw_deg": round(math.degrees(getattr(msg, "yaw", 0.0)), 2),
            "xacc": getattr(msg, "xacc", 0.0),
            "yacc": getattr(msg, "yacc", 0.0),
            "zacc": getattr(msg, "zacc", 0.0),
        }, now)

    # ------------------------------------------------------------------
    # Query
    # ------------------------------------------------------------------
    @property
    def mode(self) -> Optional[str]:
        return self.heartbeat.value[0] if self.heartbeat.value else None

    @property
    def armed(self) -> bool:
        return bool(self.heartbeat.value and self.heartbeat.value[1])

    def link_alive(self, max_age_s: float = 3.0) -> bool:
        return self.heartbeat.age(self.clock()) <= max_age_s

    def fresh_position(self, max_age_s: float = DEFAULT_MAX_AGE_S) -> PositionEstimate:
        """Return the autopilot position estimate or raise if stale/missing.

        Position is considered usable only if it is recent, and -- where the
        autopilot reports them -- the GNSS has a 3D fix and the EKF reports
        an absolute horizontal position solution.
        """
        now = self.clock()
        if self.position.value is None:
            raise StaleTelemetryError("no position received yet")
        age = self.position.age(now)
        if age > max_age_s:
            raise StaleTelemetryError("position is %.1f s old (limit %.1f s)" % (age, max_age_s))
        gnss = self.gnss.value
        if gnss is not None and self.gnss.age(now) <= 5.0 and not gnss.has_3d_fix:
            raise StaleTelemetryError("GNSS fix_type=%d (<3D)" % gnss.fix_type)
        ekf = self.ekf.value
        if ekf is not None and self.ekf.age(now) <= 5.0 and not ekf.position_ok:
            raise StaleTelemetryError("EKF horizontal position not healthy (flags=0x%x)" % ekf.flags)
        return self.position.value

    def snapshot(self) -> dict:
        """JSON-serialisable view for logging (ages in seconds)."""
        now = self.clock()

        def grp(s: Stamped):
            v = s.value
            body = v.__dict__.copy() if hasattr(v, "__dict__") else v
            return {"value": body, "age_s": None if s.rx_time is None else round(now - s.rx_time, 3)}

        return {
            "source": "autopilot_mavlink",
            "mode": self.mode, "armed": self.armed,
            "position": grp(self.position), "gnss": grp(self.gnss),
            "battery": grp(self.battery), "ekf_flags": grp(self.ekf),
            "mission_current": grp(self.mission_current),
            "sim_ground_truth": grp(self.sim_ground_truth),
            "rejected": dict(self.rejected),
        }
