#!/bin/bash
# MARIO Hardware Control - Run Script
# Controls real robot via MuJoCo + ROS2 + micro-ROS

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "========================================"
echo "  MARIO Hardware Control Setup"
echo "========================================"

# Check if Docker is running (needed for micro-ROS agent)
if ! docker info &>/dev/null; then
    echo ""
    echo "⚠️  Docker is not running!"
    echo "   Please open Docker Desktop from Applications first."
    echo "   Then re-run this script."
    echo ""
    echo "   Or run micro-ROS agent manually on a Linux machine:"
    echo "   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
    echo ""
    exit 1
fi

echo "✓ Docker is running"

# Start micro-ROS agent in Docker (background)
echo ""
echo "Starting micro-ROS agent (Docker)..."
docker run -d --rm \
    --name micro_ros_agent \
    --net=host \
    microros/micro-ros-agent:humble \
    udp4 --port 8888 \
    2>/dev/null || echo "Agent may already be running"

echo "✓ micro-ROS agent started on UDP port 8888"

# Wait for agent to be ready
sleep 2

# Run hardware control
echo ""
echo "Starting hardware control..."
echo "========================================"
echo ""

/opt/homebrew/Caskroom/miniforge/base/bin/conda run -n ros2_mario \
    python scripts/hardware_control.py

# Cleanup
echo ""
echo "Stopping micro-ROS agent..."
docker stop micro_ros_agent 2>/dev/null || true
