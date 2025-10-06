#!/bin/bash

echo "🧠 Launching MoveIt Motion Planner"
echo "=================================="
echo ""

# This script will launch MoveIt interactively so you can see any errors

docker-compose exec moveit bash -c "
    echo '📦 Sourcing ROS2...'
    source /opt/ros/humble/setup.bash
    
    echo '📦 Sourcing workspace...'
    cd /workspace
    source install/setup.bash
    
    echo '🚀 Launching MoveIt move_group...'
    echo ''
    ros2 launch ur5_moveit_config moveit.launch.py
"

