"""
flightstack/sitl.py
===================
Starts/stops an ArduPilot ArduCopter SITL process (the real autopilot
firmware compiled for the host) using ArduPilot's built-in multicopter
physics model, or an external simulator such as Gazebo via ``--model JSON``.

Locating the binary:
  * ``ARDUPILOT_HOME`` env var -> $ARDUPILOT_HOME/build/sitl/bin/arducopter
  * or ``ARDUCOPTER_SITL_BIN`` pointing directly at the binary.

Build once with (see docs/SITL_SETUP.md):
    ./waf configure --board sitl && ./waf copter
"""

from __future__ import annotations

import os
import socket
import subprocess
import tempfile
import time
from typing import List, Optional, Sequence


class SitlNotAvailable(RuntimeError):
    pass


def find_ardupilot_home() -> Optional[str]:
    home = os.environ.get("ARDUPILOT_HOME")
    if home and os.path.isdir(home):
        return os.path.abspath(home)
    # Conventional sibling checkout next to this repository.
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sib = os.path.join(os.path.dirname(here), "ardupilot")
    return os.path.abspath(sib) if os.path.isdir(sib) else None


def find_sitl_binary() -> Optional[str]:
    explicit = os.environ.get("ARDUCOPTER_SITL_BIN")
    if explicit and os.access(explicit, os.X_OK):
        return os.path.abspath(explicit)
    home = find_ardupilot_home()
    if home:
        b = os.path.join(home, "build", "sitl", "bin", "arducopter")
        if os.access(b, os.X_OK):
            return os.path.abspath(b)
    return None


def _port_open(port: int) -> bool:
    with socket.socket() as s:
        s.settimeout(0.2)
        return s.connect_ex(("127.0.0.1", port)) == 0


class SitlProcess:
    """Context manager around one ArduCopter SITL instance."""

    def __init__(self, home: Sequence[float], instance: int = 0, speedup: float = 1.0,
                 model: str = "+", extra_defaults: Optional[List[str]] = None,
                 workdir: Optional[str] = None, binary: Optional[str] = None,
                 wipe: bool = True, enable_lidar: bool = False,
                 extra_args: Optional[List[str]] = None):
        self.binary = binary or find_sitl_binary()
        if not self.binary:
            raise SitlNotAvailable("ArduCopter SITL binary not found; set ARDUPILOT_HOME")
        ap_home = find_ardupilot_home() or os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(self.binary))))
        self.defaults = [os.path.join(ap_home, "Tools", "autotest", "default_params", "copter.parm")]
        self.defaults += list(extra_defaults or [])
        self.home = home
        self.instance = instance
        self.speedup = speedup
        self.model = model
        self.wipe = wipe
        self.enable_lidar = enable_lidar
        self.extra_args = list(extra_args or [])
        self.workdir = workdir or tempfile.mkdtemp(prefix="sitl%d_" % instance)
        self.proc: Optional[subprocess.Popen] = None

    @property
    def port(self) -> int:
        return 5760 + 10 * self.instance

    @property
    def url(self) -> str:
        return "tcp:127.0.0.1:%d" % self.port

    def start(self, timeout: float = 30.0) -> "SitlProcess":
        if _port_open(self.port):
            raise SitlNotAvailable("port %d already in use (another SITL running?)" % self.port)
        lat, lon, alt, hdg = (list(self.home) + [0, 0])[:4]
        cmd = [self.binary, "--model", self.model, "--speedup", str(self.speedup),
               "--defaults", ",".join(self.defaults), "-I%d" % self.instance,
               "--home", "%f,%f,%f,%f" % (lat, lon, alt, hdg)]
        if self.wipe:
            cmd.append("-w")
        if self.enable_lidar:
            cmd.extend(["--serial5", "sim:ld06"])
        if self.extra_args:
            cmd.extend(self.extra_args)
        self.log_path = os.path.join(self.workdir, "sitl_stdout.log")
        self._log = open(self.log_path, "w")
        self.proc = subprocess.Popen(cmd, cwd=self.workdir, stdout=self._log,
                                     stderr=subprocess.STDOUT)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise SitlNotAvailable("SITL exited early; see %s" % self.log_path)
            if _port_open(self.port):
                return self
            time.sleep(0.2)
        self.stop()
        raise SitlNotAvailable("SITL did not open port %d" % self.port)

    def stop(self) -> None:
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        if getattr(self, "_log", None):
            self._log.close()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
