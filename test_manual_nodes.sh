#!/bin/bash

echo "🤖 Testing Manual Node Execution"
echo "================================"

# Start containers
echo "🚀 Starting containers..."
docker-compose up -d gazebo moveit

# Wait for startup
echo "⏳ Waiting for containers to start..."
sleep 10

echo "🔍 Testing ROS2 environment..."

# Test basic ROS2 functionality
echo "📋 Available packages:"
docker-compose exec gazebo bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | head -10"

echo ""
echo "📡 Testing our custom packages:"
docker-compose exec gazebo bash -c "source /opt/ros/humble/setup.bash && cd /workspace && source install/setup.bash && ros2 pkg list | grep ur5"

echo ""
echo "🎯 Available executables in ur5_gazebo:"
docker-compose exec gazebo bash -c "source /opt/ros/humble/setup.bash && cd /workspace && source install/setup.bash && ros2 pkg executables ur5_gazebo"

echo ""
echo "🔧 Testing individual node execution:"
echo "Starting camera detector..."
docker-compose exec gazebo bash -c "source /opt/ros/humble/setup.bash && cd /workspace && source install/setup.bash && timeout 5 ros2 run ur5_gazebo camera_detector || echo 'Node test completed'"

echo ""
echo "✅ Manual test completed!"
echo ""
echo "🚀 To enter interactive mode:"
echo "docker-compose exec gazebo bash"
echo "source /opt/ros/humble/setup.bash"
echo "cd /workspace && source install/setup.bash"