#!/bin/bash
# Convenience script to run ROS robot controller tests

set -e

# Source ROS environment
source /opt/ros/jazzy/setup.bash 2>/dev/null || echo "Warning: ROS not sourced"
source install/setup.bash 2>/dev/null || echo "Warning: Workspace not built"

echo "=========================================="
echo "ROS Robot Controller Test Suite"
echo "=========================================="
echo ""

# Default: run all static tests (no hardware needed)
if [ "$1" == "all" ]; then
    echo "Running ALL tests (including robustness - may skip without hardware)..."
    cd /workspace/src/ros_robot_controller || cd .
    python3 -m pytest test/ -v "$@"
elif [ "$1" == "quick" ]; then
    echo "Running quick tests (no hardware required)..."
    cd /workspace/src/ros_robot_controller || cd .
    python3 -m pytest test/test_service_signatures.py test/test_imports.py -v
else
    echo "Running signature and import tests..."
    cd /workspace/src/ros_robot_controller || cd .
    python3 -m pytest test/test_service_signatures.py test/test_imports.py -v
fi

echo ""
echo "=========================================="
echo "Test Results Summary"
echo "=========================================="
