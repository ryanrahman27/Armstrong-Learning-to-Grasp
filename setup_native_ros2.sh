#!/bin/bash

echo "🤖 Setting up Native ROS2 on macOS"
echo "=================================="

# Check if ROS2 is already installed
if command -v ros2 &> /dev/null; then
    echo "✅ ROS2 is already installed"
    ros2 --version
else
    echo "📦 Installing ROS2 Humble..."
    
    # Check if Homebrew is installed
    if ! command -v brew &> /dev/null; then
        echo "❌ Homebrew required. Installing..."
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    fi
    
    # Add ROS2 tap and install
    brew tap ros/robot_localization
    brew install ros-humble-desktop
fi

echo ""
echo "🔧 To use native ROS2 with Armstrong:"
echo "1. Source ROS2: source /opt/homebrew/bin/ros2"
echo "2. Start containers (without RViz): docker-compose up gazebo moveit"
echo "3. Run native RViz: ros2 launch ur5_moveit_config rviz.launch.py"
echo ""
echo "Note: You'll need to install additional packages:"
echo "brew install ros-humble-moveit ros-humble-rviz2"