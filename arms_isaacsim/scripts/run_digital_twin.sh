#!/bin/bash
# Run the ARMS digital twin (Isaac Sim + ROS2 action servers).
#
# Usage (from anywhere):
#   bash arms_isaacsim/scripts/run_digital_twin.sh [--headless] [options]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# scripts/ → arms_isaacsim/ → arms_ros2_ws/
WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "[run_digital_twin] Workspace: ${WS_ROOT}"

# Source ROS2 and the built workspace
source /opt/ros/jazzy/setup.bash
source "${WS_ROOT}/install/setup.bash"

# Write the system Python's sys.path to a temp file.
# Isaac Sim's venv clears PYTHONPATH from os.environ at startup, so we
# can't pass paths via environment variables reliably.  A file survives.
ROS2_PATHS_FILE="/tmp/arms_ros2_paths_$$.txt"
python3 -c "import sys; print('\n'.join(p for p in sys.path if p))" \
    > "${ROS2_PATHS_FILE}"

echo "[run_digital_twin] Wrote $(wc -l < "${ROS2_PATHS_FILE}") system Python paths to ${ROS2_PATHS_FILE}"
export ARMS_ROS2_PATHS_FILE="${ROS2_PATHS_FILE}"

# Clean up temp file on exit
trap 'rm -f "${ROS2_PATHS_FILE}"' EXIT

exec ~/isaacsim_env/bin/python "${SCRIPT_DIR}/digital_twin.py" "$@"
