#!/usr/bin/env bash
# ==============================================================================
# setup_env.sh — Med-Drone Mission Control Environment Setup & Diagnostic Tool
# ==============================================================================
# Usage:
#   source setup_env.sh           (to activate environment in current shell)
#   ./setup_env.sh                (to verify environment and dependencies)
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}===================================================================${NC}"
echo -e "${BLUE}  Med-Drone Mission Control — Environment Setup & Diagnostics       ${NC}"
echo -e "${BLUE}===================================================================${NC}"

# 1. Detect Python 3
PYTHON_BIN=""
if command -v python3 &>/dev/null; then
    PYTHON_BIN="python3"
elif command -v python &>/dev/null; then
    PYTHON_BIN="python"
else
    echo -e "${RED}[ERROR] Neither python3 nor python found in PATH.${NC}"
    exit 1
fi

PY_VER=$($PYTHON_BIN -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo -e "[✓] System Python detected: ${GREEN}${PYTHON_BIN} (v${PY_VER})${NC}"

# 2. Virtual Environment Setup
VENV_DIR="../.venv-sitl"
if [ ! -d "$VENV_DIR" ]; then
    echo -e "${YELLOW}[*] Creating virtual environment at: ${VENV_DIR}${NC}"
    $PYTHON_BIN -m venv "$VENV_DIR"
fi

VENV_PY="${VENV_DIR}/bin/python"
VENV_PIP="${VENV_DIR}/bin/pip"

if [ -f "$VENV_PY" ]; then
    echo -e "[✓] Virtual environment ready: ${GREEN}${VENV_DIR}${NC}"
else
    echo -e "${RED}[ERROR] Failed to locate virtual environment python at ${VENV_PY}${NC}"
    exit 1
fi

# 3. Dependencies
echo -e "${YELLOW}[*] Checking Python dependencies...${NC}"
if [ -f "requirements-sitl.txt" ]; then
    "$VENV_PIP" install -q -r requirements-sitl.txt
    echo -e "[✓] SITL dependencies verified from requirements-sitl.txt"
fi

# Optional dependencies check
"$VENV_PY" -c "import pymavlink; print('[✓] pymavlink is installed (version: ' + pymavlink.__version__ + ')')" 2>/dev/null || echo -e "${YELLOW}[!] pymavlink not installed in venv.${NC}"
"$VENV_PY" -c "import cv2; print('[✓] OpenCV (cv2) is installed (version: ' + cv2.__version__ + ')')" 2>/dev/null || echo -e "${YELLOW}[!] OpenCV not installed (visual landing markers will use fallback math).${NC}"
"$VENV_PY" -c "import folium; print('[✓] folium is installed')" 2>/dev/null || echo -e "${YELLOW}[!] folium not installed (simulation_replay.html will use standard Leaflet visualizer).${NC}"

# 4. Check ArduPilot SITL binary
ARDUPILOT_DIRS=("../ardupilot" "../../ardupilot" "$HOME/ardupilot")
SITL_BIN=""

if [ -n "$ARDUPILOT_HOME" ] && [ -f "$ARDUPILOT_HOME/build/sitl/bin/arducopter" ]; then
    SITL_BIN="$ARDUPILOT_HOME/build/sitl/bin/arducopter"
else
    for d in "${ARDUPILOT_DIRS[@]}"; do
        if [ -f "$d/build/sitl/bin/arducopter" ]; then
            SITL_BIN="$(cd "$d/build/sitl/bin" && pwd)/arducopter"
            export ARDUPILOT_HOME="$(cd "$d" && pwd)"
            break
        fi
    done
fi

if [ -n "$SITL_BIN" ]; then
    echo -e "[✓] ArduCopter SITL binary found: ${GREEN}${SITL_BIN}${NC}"
    echo -e "[✓] ARDUPILOT_HOME set to: ${GREEN}${ARDUPILOT_HOME}${NC}"
else
    echo -e "${YELLOW}[!] ArduCopter SITL binary not found in standard paths.${NC}"
    echo -e "    Pure Python algorithms, VRP routing, physics models, and replay dashboard remain 100% operational."
    echo -e "    To compile SITL for macOS, see: docs/SITL_SETUP.md"
fi

# 5. Check port 5760 availability
if command -v lsof &>/dev/null; then
    PORT_BUSY=$(lsof -i :5760 2>/dev/null || true)
    if [ -n "$PORT_BUSY" ]; then
        echo -e "${YELLOW}[!] Port 5760 is currently occupied by another process.${NC}"
        echo -e "    Kill with: lsof -ti:5760 | xargs kill -9"
    else
        echo -e "[✓] SITL TCP port 5760 is free."
    fi
fi

echo -e "${BLUE}===================================================================${NC}"
echo -e "${GREEN}Running comprehensive capability diagnostics...${NC}"
"$VENV_PY" check_env.py
