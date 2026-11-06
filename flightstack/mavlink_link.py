"""
flightstack/mavlink_link.py
===========================
Thin MAVLink client for an ArduPilot autopilot (SITL or hardware), built on
pymavlink. Responsibilities:

  * one background reader thread that feeds every message into
    ``TelemetryState`` and routes protocol replies (MISSION_REQUEST*,
    MISSION_ACK, COMMAND_ACK) to a queue;
  * message-rate requests for the telemetry the planner needs;
  * the MAVLink mission-upload micro-protocol (COUNT -> REQUEST_INT* ->
    ITEM_INT* -> ACK) with per-item timeouts and retries;
  * COMMAND_LONG helpers (arm, mode change, mission start) that wait for
    COMMAND_ACK and report failures instead of assuming success.

Flight control and state estimation stay inside the autopilot; nothing here
closes a control loop.
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Callable, Iterable, List, Optional

from flightstack.mission import MissionItem
from flightstack.telemetry import TelemetryState, COPTER_MODES

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover - exercised only without pymavlink
    mavutil = None

_PROTOCOL_TYPES = {"MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK",
                   "COMMAND_ACK", "MISSION_COUNT", "MISSION_ITEM_INT"}

MODE_IDS = {name: num for num, name in COPTER_MODES.items()}

# MAVLink message ids for SET_MESSAGE_INTERVAL
_STREAMS_HZ = {
    33: 5.0,    # GLOBAL_POSITION_INT
    24: 2.0,    # GPS_RAW_INT
    1: 2.0,     # SYS_STATUS
    147: 1.0,   # BATTERY_STATUS
    193: 1.0,   # EKF_STATUS_REPORT
    42: 2.0,    # MISSION_CURRENT
    74: 2.0,    # VFR_HUD
    132: 2.0,   # DISTANCE_SENSOR (AP_Proximity sector feedback)
    246: 5.0,   # ADSB_VEHICLE (AP_OADatabase obstacle feedback, callsign="OA_DB")
}


class LinkError(RuntimeError):
    pass


class AutopilotLink:
    def __init__(self, conn, telemetry: Optional[TelemetryState] = None,
                 on_message: Optional[Callable] = None):
        """``conn`` is a pymavlink connection (or a test double exposing
        ``recv_match``, ``mav`` and ``target_system/target_component``)."""
        self.conn = conn
        self.telemetry = telemetry or TelemetryState()
        self.on_message = on_message
        self._proto: "queue.Queue" = queue.Queue()
        self._stop = threading.Event()
        self._reader: Optional[threading.Thread] = None
        self._proximity_thread: Optional[threading.Thread] = None
        self._proximity_distances: List[int] = [65535] * 72
        self._send_lock = threading.Lock()
        self._is_live_socket: bool = False

    # ------------------------------------------------------------------
    @classmethod
    def connect(cls, url: str = "tcp:127.0.0.1:5760", heartbeat_timeout: float = 30.0,
                **kw) -> "AutopilotLink":
        if mavutil is None:
            raise LinkError("pymavlink is not installed (pip install -r requirements-sitl.txt)")
        deadline = time.time() + heartbeat_timeout
        last_err = None
        while time.time() < deadline:
            try:
                conn = mavutil.mavlink_connection(url, source_system=255, source_component=190)
                break
            except OSError as e:          # SITL not listening yet
                last_err = e
                time.sleep(0.5)
        else:
            raise LinkError("could not open %s: %s" % (url, last_err))
        hb = conn.wait_heartbeat(timeout=max(1.0, deadline - time.time()))
        if hb is None:
            raise LinkError("no autopilot heartbeat on %s" % url)
        link = cls(conn, **kw)
        link._is_live_socket = True
        link.telemetry.update(hb)
        link.start()
        link.request_streams()
        return link

    def start(self) -> None:
        self._reader = threading.Thread(target=self._read_loop, name="mavlink-reader", daemon=True)
        self._reader.start()
        if not self._is_live_socket:
            self._proximity_thread = threading.Thread(target=self._proximity_loop, name="mavlink-proximity", daemon=True)
            self._proximity_thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._proximity_thread:
            self._proximity_thread.join(timeout=1.0)
        if self._reader:
            self._reader.join(timeout=2.0)
        try:
            self.conn.close()
        except Exception:
            pass

    def send_obstacle_distance(self, distances: List[int]) -> None:
        """Update and immediately transmit 72-bin MAVLink OBSTACLE_DISTANCE (cm)."""
        if len(distances) == 72:
            self._proximity_distances = list(distances)
        mav = getattr(self.conn, "mav", None)
        if mav is not None and hasattr(mav, "obstacle_distance_send"):
            time_us = int(time.time() * 1e6)
            try:
                with self._send_lock:
                    mav.obstacle_distance_send(
                        time_us, 0, self._proximity_distances, 5, 20, 3500, 5.0, 0.0, 12
                    )
            except Exception:
                pass

    def _proximity_loop(self) -> None:
        """Continuously stream OBSTACLE_DISTANCE at ~33 Hz wall-clock so ArduPilot
        PRX1_TYPE=2 stays healthy even at 10x SITL speedup (500 ms sim timeout)."""
        while not self._stop.is_set():
            self.send_obstacle_distance(self._proximity_distances)
            self._stop.wait(0.03)

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.2)
            except Exception:
                if self._stop.is_set():
                    return
                time.sleep(0.05)
                continue
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == "BAD_DATA":
                self.telemetry._reject("bad_data")
                continue
            # Only trust messages from the autopilot we are talking to.
            src = getattr(msg, "get_srcSystem", lambda: self.conn.target_system)()
            if src not in (self.conn.target_system, 0):
                continue
            self.telemetry.update(msg)
            if mtype in _PROTOCOL_TYPES:
                self._proto.put(msg)
            if self.on_message:
                self.on_message(msg)

    # ------------------------------------------------------------------
    @property
    def _ts(self):
        return self.conn.target_system

    @property
    def _tc(self):
        return self.conn.target_component

    def _wait_proto(self, types: Iterable[str], timeout: float, pred=None):
        types = set(types)
        deadline = time.time() + timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None
            try:
                msg = self._proto.get(timeout=remaining)
            except queue.Empty:
                return None
            if msg.get_type() in types and (pred is None or pred(msg)):
                return msg

    def _drain_proto(self) -> None:
        while True:
            try:
                self._proto.get_nowait()
            except queue.Empty:
                return

    # ------------------------------------------------------------------
    def request_streams(self) -> None:
        for msg_id, hz in _STREAMS_HZ.items():
            self.command_long(511, msg_id, 1e6 / hz, wait_ack=False)

    def command_long(self, command: int, *params: float, wait_ack: bool = True,
                     timeout: float = 3.0, retries: int = 2) -> Optional[int]:
        p = list(params) + [0.0] * (7 - len(params))
        for attempt in range(retries + 1):
            with self._send_lock:
                self.conn.mav.command_long_send(self._ts, self._tc, command, attempt, *p)
            if not wait_ack:
                return None
            ack = self._wait_proto(["COMMAND_ACK"], timeout, lambda m: m.command == command)
            if ack is not None:
                return int(ack.result)
        raise LinkError("no COMMAND_ACK for command %d" % command)

    def set_mode(self, mode: str, timeout: float = 5.0) -> None:
        if mode not in MODE_IDS:
            raise ValueError("unknown copter mode %s" % mode)
        res = self.command_long(176, 1, MODE_IDS[mode])   # MAV_CMD_DO_SET_MODE
        if res != 0:
            raise LinkError("mode change to %s rejected (result %s)" % (mode, res))
        self.wait_for(lambda t: t.mode == mode, timeout, "mode %s" % mode)

    def arm(self, timeout: float = 60.0) -> None:
        """Arm, retrying while pre-arm checks (EKF/GPS settling) fail."""
        deadline = time.time() + timeout
        last = None
        while time.time() < deadline:
            last = self.command_long(400, 1)              # MAV_CMD_COMPONENT_ARM_DISARM
            if last == 0:
                self.wait_for(lambda t: t.armed, 5.0, "armed")
                return
            time.sleep(2.0)
        msgs = [t for _, _, t in self.telemetry.status_texts[-5:]]
        raise LinkError("arming rejected (result %s); recent autopilot text: %s" % (last, msgs))

    def start_mission(self) -> None:
        res = self.command_long(300, 0, 0)                # MAV_CMD_MISSION_START
        if res != 0:
            raise LinkError("MISSION_START rejected (result %s)" % res)

    def set_current_item(self, seq: int) -> None:
        with self._send_lock:
            self.conn.mav.mission_set_current_send(self._ts, self._tc, seq)

    def set_param(self, name: str, value: float, timeout: float = 3.0) -> None:
        with self._send_lock:
            self.conn.mav.param_set_send(self._ts, self._tc, name.encode(), float(value), 9)

    def wait_for(self, predicate: Callable[[TelemetryState], bool], timeout: float,
                 what: str = "condition") -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate(self.telemetry):
                return
            time.sleep(0.1)
        raise LinkError("timed out after %.0f s waiting for %s" % (timeout, what))

    def wait_ready(self, timeout: float = 90.0) -> None:
        """Block until the autopilot reports a usable navigation solution.

        ArduPilot rejects mission uploads ("Only 0 items are supported",
        MAV_MISSION_NO_SPACE) until its mission storage is initialised at
        boot, and refuses to arm until the EKF has GNSS-aided position.
        """
        from flightstack.telemetry import StaleTelemetryError

        def ok(t):
            try:
                t.fresh_position()
            except StaleTelemetryError:
                return False
            return t.ekf.value is not None and t.gnss.value is not None
        self.wait_for(ok, timeout, "EKF position + 3D GNSS fix")
        if self._is_live_socket:
            pos = self.telemetry.position.value
            if pos is not None:
                try:
                    self.upload_fence_obstacles((pos.lat, pos.lon), [])
                    time.sleep(0.25)
                except Exception:
                    pass

    def upload_fence_obstacles(
        self,
        home: tuple,
        obstacles: Iterable[tuple] = (),
        inclusion_half_deg: float = 0.10,
        item_timeout: float = 2.0,
        retries: int = 5,
    ) -> int:
        """Upload world-geometry inclusion boundary and cylindrical exclusion obstacles
        via MAVLink Fence Protocol (mission_type = 1, MAV_MISSION_TYPE_FENCE).

        ArduPilot's native C++ ``AP_Proximity_SITL`` (``PRX1_TYPE = 10``) raycasts 8
        sectors in C++ against ``AP::fence()->polyfence()``. It requires at least one
        polygonal inclusion boundary (``MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001``)
        to report sensor health ``Status::Good``, and raycasts against any circular
        exclusion obstacles (``MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004``) placed in
        the simulated world.
        """
        h_lat, h_lon = float(home[0]), float(home[1])
        d = float(inclusion_half_deg)
        box = [
            (h_lat - d, h_lon - d),
            (h_lat - d, h_lon + d),
            (h_lat + d, h_lon + d),
            (h_lat + d, h_lon - d),
        ]
        # Each tuple: (command, p1, lat, lon)
        fence_items = [(5001, 4.0, lat, lon) for lat, lon in box]
        for obs_lat, obs_lon, radius_m in obstacles:
            fence_items.append((5004, float(radius_m), float(obs_lat), float(obs_lon)))

        self._drain_proto()
        n = len(fence_items)
        for _ in range(retries + 1):
            with self._send_lock:
                self.conn.mav.mission_count_send(self._ts, self._tc, n, 1)
            first = self._wait_proto(
                ["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                item_timeout,
            )
            if first is not None:
                break
        else:
            raise LinkError("autopilot never requested fence items")

        msg = first
        sent = set()
        stalls = 0
        while True:
            if msg is None:
                stalls += 1
                if stalls > retries:
                    raise LinkError("fence upload stalled after items %s" % sorted(sent))
                msg = self._wait_proto(
                    ["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                    item_timeout,
                )
                continue
            mtype = msg.get_type()
            if mtype == "MISSION_ACK":
                if int(msg.type) != 0:
                    raise LinkError("fence rejected: MAV_MISSION_RESULT %d" % msg.type)
                break
            seq = int(msg.seq)
            if not 0 <= seq < n:
                raise LinkError("autopilot requested invalid fence seq %d" % seq)
            cmd, p1, lat, lon = fence_items[seq]
            with self._send_lock:
                self.conn.mav.mission_item_int_send(
                    self._ts, self._tc, seq, 0, cmd,
                    0, 1,
                    float(p1), 0.0, 0.0, 0.0,
                    int(round(lat * 1e7)), int(round(lon * 1e7)), 0.0, 1,
                )
            sent.add(seq)
            stalls = 0
            msg = self._wait_proto(
                ["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                item_timeout,
            )
        return len(fence_items) - 4

    # ------------------------------------------------------------------
    def upload_mission(self, items: List[MissionItem], item_timeout: float = 2.0,
                       retries: int = 5, verify: bool = False) -> None:
        """MAVLink mission upload protocol (mission_type 0 = MISSION)."""
        if not items:
            raise ValueError("empty mission")
        self._drain_proto()
        n = len(items)
        for attempt in range(retries + 1):
            with self._send_lock:
                self.conn.mav.mission_count_send(self._ts, self._tc, n, 0)
            first = self._wait_proto(["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                                     item_timeout)
            if first is not None:
                break
        else:
            raise LinkError("autopilot never requested mission items")

        msg = first
        sent = set()
        stalls = 0
        while True:
            if msg is None:
                stalls += 1
                if stalls > retries:
                    raise LinkError("mission upload stalled after items %s" % sorted(sent))
                msg = self._wait_proto(["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                                       item_timeout)
                continue
            mtype = msg.get_type()
            if mtype == "MISSION_ACK":
                if int(msg.type) != 0:
                    raise LinkError("mission rejected: MAV_MISSION_RESULT %d" % msg.type)
                if len(sent) != n:
                    raise LinkError("ACK before all items sent (%d/%d)" % (len(sent), n))
                break
            seq = int(msg.seq)
            if not 0 <= seq < n:
                raise LinkError("autopilot requested invalid seq %d" % seq)
            it = items[seq]
            with self._send_lock:
                self.conn.mav.mission_item_int_send(
                    self._ts, self._tc, seq, it.frame, it.command,
                    0, it.autocontinue,
                    float(it.params[0]), float(it.params[1]),
                    float(it.params[2]), float(it.params[3]),
                    int(round(it.lat * 1e7)), int(round(it.lon * 1e7)), float(it.alt_m), 0)
            sent.add(seq)
            stalls = 0
            msg = self._wait_proto(["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                                   item_timeout)

        if verify:
            self.verify_mission_readback(items, timeout=item_timeout * 2)

    def download_mission_count(self, timeout: float = 3.0) -> int:
        self._drain_proto()
        with self._send_lock:
            self.conn.mav.mission_request_list_send(self._ts, self._tc, 0)
        msg = self._wait_proto(["MISSION_COUNT"], timeout)
        if msg is None:
            raise LinkError("no MISSION_COUNT reply")
        return int(msg.count)

    def download_mission(self, timeout: float = 3.0) -> List[tuple]:
        """Read back the stored mission as (seq, command, lat, lon, alt) tuples."""
        n = self.download_mission_count(timeout)
        out = []
        for seq in range(n):
            with self._send_lock:
                self.conn.mav.mission_request_int_send(self._ts, self._tc, seq, 0)
            msg = self._wait_proto(["MISSION_ITEM_INT"], timeout, lambda m, s=seq: m.seq == s)
            if msg is None:
                raise LinkError("no MISSION_ITEM_INT for seq %d" % seq)
            out.append((seq, msg.command, msg.x / 1e7, msg.y / 1e7, msg.z))
        with self._send_lock:
            self.conn.mav.mission_ack_send(self._ts, self._tc, 0, 0)
        return out

    def verify_mission_readback(self, expected_items: List[MissionItem], timeout: float = 3.0) -> bool:
        """Verify the stored autopilot mission matches the expected items item-by-item."""
        downloaded = self.download_mission(timeout=timeout)
        if len(downloaded) != len(expected_items):
            raise LinkError(f"Mission readback item count mismatch: expected {len(expected_items)}, got {len(downloaded)}")
        for seq, (d_seq, d_cmd, d_lat, d_lon, d_alt) in enumerate(downloaded):
            if seq == 0:
                # Seq 0 is ArduPilot home position; autopilot automatically converts its
                # altitude to the home elevation AMSL (e.g. 431m), while nav items start from seq 1.
                continue
            exp = expected_items[seq]
            if d_cmd != exp.command:
                raise LinkError(f"Mission command mismatch at seq {seq}: expected {exp.command}, read back {d_cmd}")
            if exp.lat != 0.0 or exp.lon != 0.0:
                if abs(d_lat - exp.lat) > 1e-5 or abs(d_lon - exp.lon) > 1e-5:
                    raise LinkError(f"Mission coordinate mismatch at seq {seq}: expected ({exp.lat:.6f}, {exp.lon:.6f}), read back ({d_lat:.6f}, {d_lon:.6f})")
            if abs(d_alt - exp.alt_m) > 0.5:
                raise LinkError(f"Mission altitude mismatch at seq {seq}: expected {exp.alt_m:.1f}m, read back {d_alt:.1f}m")
        return True
