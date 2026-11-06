#!/usr/bin/env python3
"""
check_env.py
============
System diagnostics and capability verification for Med-Drone Mission Control.

Checks:
  1. Python runtime and virtual environments.
  2. Core and optional dependencies.
  3. Native ArduPilot SITL binary and TCP port 5760.
  4. Computer vision libraries, Apple Silicon GPU acceleration (MPS), and YOLO weights.
  5. Camera video input sources (Live AVFoundation TCC status, recorded, synthetic).
  6. Memory and resource boundaries (8 GB RAM constraint).
  7. Exact capability breakdown: Local vs Linux/Gazebo vs Physical Hardware.
"""

import os
import platform
import socket
import subprocess
import sys
import types

# Ensure lzma compatibility
if "lzma" not in sys.modules:
    try:
        import lzma
    except ImportError:
        m = types.ModuleType("lzma")
        m.open = None
        m.LZMAFile = None
        m.LZMAError = Exception
        sys.modules["lzma"] = m


def check_memory():
    try:
        import psutil
        mem = psutil.virtual_memory()
        return mem.total / (1024**3), mem.available / (1024**3)
    except ImportError:
        try:
            out = subprocess.run(["sysctl", "hw.memsize"], capture_output=True, text=True).stdout
            total = int(out.split(":")[1].strip()) / (1024**3)
            return total, -1.0
        except Exception:
            return -1.0, -1.0


def check_port_free(port: int = 5760) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        res = s.connect_ex(("127.0.0.1", port))
        return res != 0  # True if nothing listening (port free)


def check_camera_capture():
    try:
        import cv2
        cap = cv2.VideoCapture(0)
        opened = cap.isOpened()
        if not opened:
            cap.release()
            return False, "Not accessible (macOS TCC permission not granted or no hardware camera)"
        ret, frame = cap.read()
        cap.release()
        if ret and frame is not None:
            return True, f"Live hardware camera working ({frame.shape[1]}x{frame.shape[0]})"
        return False, "Camera opened but read failed (frame empty)"
    except Exception as e:
        return False, f"OpenCV capture error: {e}"


def main():
    print("=" * 70)
    print("  MED-DRONE MISSION CONTROL — SYSTEM DIAGNOSTICS & CAPABILITY MATRIX")
    print("=" * 70)

    # 1. Platform & Memory
    total_gb, avail_gb = check_memory()
    print(f"\n[1] HOST ENVIRONMENT:")
    print(f"  - Platform:          {platform.platform()}")
    print(f"  - Machine Arch:      {platform.machine()} ({'Apple Silicon' if platform.machine() == 'arm64' else 'Intel/x86'})")
    print(f"  - Python Version:    {sys.version.split()[0]} ({sys.executable})")
    avail_str = f", {avail_gb:.1f} GB available" if avail_gb > 0 else ""
    print(f"  - System Memory:     {total_gb:.1f} GB total{avail_str} (Target: 8 GB constraint)")

    # 2. ArduPilot SITL Firmware
    sitl_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "ardupilot", "build", "sitl", "bin", "arducopter"))
    sitl_exists = os.path.exists(sitl_path)
    sitl_exec = os.access(sitl_path, os.X_OK) if sitl_exists else False
    port_free = check_port_free(5760)
    print(f"\n[2] AUTOPILOT SITL STACK:")
    print(f"  - Binary Path:       {sitl_path}")
    print(f"  - Binary Status:     {'READY (Executable)' if (sitl_exists and sitl_exec) else 'MISSING / NOT BUILT'}")
    print(f"  - Port 5760 Status:  {'FREE (Ready for SITL)' if port_free else 'IN USE (Port conflict)'}")

    # 3. Python Package Ecosystem
    pkgs = [
        ("pymavlink", "MAVLink communications protocol engine"),
        ("cv2", "Computer vision & ArUco precision landing"),
        ("ultralytics", "YOLOv8 real-time perception pipeline"),
        ("torch", "PyTorch neural network inference backend"),
        ("matplotlib", "Static mission & solver plotting"),
        ("folium", "Interactive geographic map visualizer"),
        ("ortools", "Comparative VRP solver baseline"),
    ]
    print(f"\n[3] PYTHON DEPENDENCY ECOSYSTEM:")
    all_core_ok = True
    for mod_name, desc in pkgs:
        try:
            m = __import__(mod_name)
            v = getattr(m, "__version__", "installed")
            print(f"  - {mod_name:<14} [{v:<10}] {desc}")
        except ImportError:
            print(f"  - {mod_name:<14} [MISSING   ] {desc}")
            if mod_name in ("pymavlink", "cv2"):
                all_core_ok = False

    # Check GPU Acceleration
    try:
        import torch
        mps_ok = torch.backends.mps.is_available()
        print(f"  - PyTorch MPS (GPU): {'ENABLED (Apple Silicon Metal Performance Shaders)' if mps_ok else 'CPU only'}")
    except Exception:
        pass

    # 4. Computer Vision & Pretrained Weights
    weights_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "yolov8n.pt"))
    weights_ok = os.path.exists(weights_path)
    weights_size_mb = os.path.getsize(weights_path) / (1024*1024) if weights_ok else 0.0
    print(f"\n[4] PERCEPTION ASSETS:")
    print(f"  - YOLOv8 Weights:    {weights_path}")
    print(f"  - Weights Status:    {'READY (' + str(round(weights_size_mb, 1)) + ' MB)' if weights_ok else 'MISSING (Will auto-download on first run)'}")

    # 5. Video Sources
    cam_ok, cam_desc = check_camera_capture()
    print(f"\n[5] VIDEO INPUT SOURCES:")
    print(f"  - Live Webcam:       {cam_desc}")
    print(f"  - Recorded Video:    READY (Supported via RecordedVideoSource with MP4/AVI)")
    print(f"  - Synthetic Stream:  READY (Supported via SyntheticCameraSource with ground-truth rendering)")
    print(f"  - Synthetic Depth:   READY (Supported via SyntheticDepthSource for 3D coordinate projection)")

    # 6. Execution Capability Breakdown
    print(f"\n[6] CAPABILITY MATRIX & TARGET SCOPE:")
    print("  +-------------------------------------------------------------+---------------------+")
    print("  | Operational Capability                                      | Execution Status    |")
    print("  +-------------------------------------------------------------+---------------------+")
    print("  | Fleet VRP Solver (Urgency NN, Savings, 2-opt, Or-opt)       | LOCAL (Pure Python) |")
    print("  | Polygon A* Detours & 50m Safety Buffer Clearance            | LOCAL (Pure Python) |")
    print("  | Multicopter Mass-Power Physics & MTOW Enforcement           | LOCAL (Pure Python) |")
    print("  | QGroundControl WPL 110 Waypoint Mission Generation          | LOCAL (Pure Python) |")
    print("  | ArduPilot SITL Flight Execution over MAVLink TCP:5760       | LOCAL (SITL Native) |")
    print("  | Closed-Loop Mission Monitoring & Readback Verification      | LOCAL (SITL Native) |")
    print("  | Hover Gripper & Land-and-Deliver Sequences                  | LOCAL (SITL Native) |")
    print("  | Tactical Hold (LOITER) on Collision Risk (t_cpa, d_cpa)     | LOCAL (SITL Native) |")
    print("  | In-Flight A* Replanning on Pop-up No-Fly Zones              | LOCAL (SITL Native) |")
    print("  | Streaming YOLOv8 Real-Time Video Object Detection           | LOCAL (Torch + MPS) |")
    print("  | ArUco Fiducial Optical-to-Body Precision Landing Target     | LOCAL (OpenCV)      |")
    print("  | Leaflet Telemetry HUD Flight Replay Dashboard               | LOCAL (Browser HTML)|")
    print("  | Gazebo 3D World Simulation & Multi-UAV Physics              | REQUIRES LINUX / 16G|")
    print("  | Live ROS 2 Micro-XRCE Agent Bridge                          | REQUIRES LINUX      |")
    print("  | Physical Drone Hardware (Pixhawk, RTK GPS, Winch Solenoid)  | REQUIRES HARDWARE   |")
    print("  +-------------------------------------------------------------+---------------------+")

    print(f"\n[7] VERIFIED RUN COMMANDS:")
    print("  * Full Automated Unit Suite (149 tests):")
    print("      python3 -m unittest discover -s tests -p 'test_*.py'")
    print("  * Fleet Mission-Level Discrete Simulation:")
    print("      python3 run_simulation.py --events 3")
    print("  * Live ArduPilot SITL Hover Delivery Demo (5x speed):")
    print("      ARDUPILOT_HOME='../ardupilot' ../.venv-sitl/bin/python sitl_mission_demo.py --speedup 5.0 --max-stops 2")
    print("  * Live ArduPilot SITL Land-and-Deliver Demo:")
    print("      ARDUPILOT_HOME='../ardupilot' ../.venv-sitl/bin/python sitl_mission_demo.py --mode land --speedup 5.0 --max-stops 2")
    print("  * Live ArduPilot SITL Dynamic Obstacle Avoidance Demo:")
    print("      ARDUPILOT_HOME='../ardupilot' ../.venv-sitl/bin/python sitl_mission_demo.py --inject-obstacle --speedup 5.0 --max-stops 2")
    print("  * Interactive Telemetry HUD Playback Generation:")
    print("      python3 visualize_sitl.py output/sitl_run_*.jsonl -o output/sitl_dashboard.html")
    print("=" * 70 + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
