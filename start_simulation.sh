#!/bin/bash

echo "🎮 Starting Armstrong Simulation"
echo "==============================="

# Stop any existing containers
docker-compose down

echo "🚀 Starting container without auto-launch..."

# Start container without the problematic auto-launch
docker run -it --rm \
    --name armstrong_sim \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd)/workspace:/workspace \
    -v $(pwd)/data:/data \
    osrf/ros:humble-desktop-full \
    bash -c "
        # Install dependencies
        apt-get update && apt-get install -y \
            ros-humble-gazebo-ros-pkgs \
            ros-humble-gazebo-ros \
            ros-humble-ur-description \
            ros-humble-moveit \
            ros-humble-joint-state-publisher \
            ros-humble-robot-state-publisher \
            ros-humble-xacro

        # Source ROS2
        source /opt/ros/humble/setup.bash
        
        echo '✅ ROS2 and Gazebo are ready!'
        echo 'ROS2 Commands available:'
        echo '  ros2 run gazebo_ros gazebo'
        echo '  ros2 launch gazebo_ros gazebo.launch.py'
        echo '  ros2 topic list'
        echo '  ros2 node list'
        echo ''
        echo '🎮 Starting interactive shell...'
        
        # Start interactive shell
        bash
    "