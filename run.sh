#!/bin/bash
# Quick-run the dashboard without colcon build.
# Usage: ./run.sh
#
# Make sure ROS2 is sourced and your robot's topics are reachable:
#   source /opt/ros/humble/setup.bash
#   export ROS_DOMAIN_ID=<your_domain_id>  # if needed

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS if not already
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi

echo "════════════════════════════════════════════"
echo "  NAVDASH — Navigation Dashboard"
echo "  Open http://localhost:8420 in your browser"
echo "════════════════════════════════════════════"
echo ""

cd "$SCRIPT_DIR"
PYTHONPATH="$SCRIPT_DIR/src:$PYTHONPATH" python3 -m nav_dashboard.main
