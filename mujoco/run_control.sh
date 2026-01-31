#!/bin/bash
# Run hardware control (assumes micro-ROS agent is already running elsewhere)

cd "$(dirname "$0")"

echo "MARIO Hardware Control"
echo "======================"
echo "Make sure micro-ROS agent is running:"
echo "  docker run --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888"
echo ""

/opt/homebrew/Caskroom/miniforge/base/bin/conda run -n ros2_mario \
    python scripts/hardware_control.py
