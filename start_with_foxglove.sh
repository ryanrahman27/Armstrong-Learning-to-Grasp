#!/bin/bash

echo "🦊 Starting Armstrong with Foxglove Support"
echo "==========================================="
echo ""

# Stop any existing containers
echo "🛑 Stopping existing containers..."
docker-compose down

# Start the containers
echo "🚀 Starting containers (this may take a few minutes on first run)..."
docker-compose up -d gazebo moveit

# Wait for containers to be ready
echo "⏳ Waiting for containers to start..."
sleep 10

# Check if containers are running
if ! docker ps | grep -q "armstrong_gazebo"; then
    echo "❌ Error: Gazebo container failed to start"
    exit 1
fi

echo "✅ Containers are running!"
echo ""

# Wait for packages to install
echo "⏳ Waiting for packages to install (this takes time on first run)..."
sleep 20

# Rebuild the workspace with Foxglove support
echo "🔨 Building workspace with Foxglove support..."
docker-compose exec -T gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    colcon build --packages-select ur5_gazebo ur5_moveit_config &&
    source install/setup.bash &&
    echo 'Build complete!'
"

echo ""
echo "✅ Setup complete! Now launching simulation..."
echo ""

# Launch the simulation in the background
echo "🎮 Starting simulation with Foxglove bridge..."
docker-compose exec -d gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    source install/setup.bash &&
    ros2 launch ur5_gazebo ur5.launch.py
"

echo ""
echo "⏳ Waiting for nodes to start..."
sleep 5

# Check if Foxglove bridge is running
echo ""
echo "🔍 Checking if Foxglove bridge is running..."
docker-compose exec -T gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 node list | grep foxglove
" && echo "✅ Foxglove bridge is running!" || echo "⚠️  Foxglove bridge may not be running yet"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🎉 SUCCESS! Your simulation is running!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📊 To visualize in Foxglove Studio:"
echo "   1. Download Foxglove Studio: https://foxglove.dev/download"
echo "   2. Or use the web version: https://app.foxglove.dev"
echo "   3. Click 'Open Connection'"
echo "   4. Select 'Rosbridge (ROS 1 & 2)'"
echo "   5. Enter: ws://localhost:8765"
echo "   6. Click 'Open'"
echo ""
echo "🔍 View running nodes:"
echo "   docker-compose exec gazebo bash -c 'source /opt/ros/humble/setup.bash && ros2 node list'"
echo ""
echo "📡 View available topics:"
echo "   docker-compose exec gazebo bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list'"
echo ""
echo "📋 View logs:"
echo "   docker-compose logs -f gazebo"
echo ""
echo "🛑 Stop everything:"
echo "   docker-compose down"
echo ""
echo "📖 For more details, see: FOXGLOVE_SETUP.md"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

