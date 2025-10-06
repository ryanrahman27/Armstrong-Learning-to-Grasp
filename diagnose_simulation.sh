#!/bin/bash

echo "🔍 Armstrong Simulation Diagnostics"
echo "===================================="
echo ""

# Check if container is running
echo "1️⃣  Checking if containers are running..."
if docker ps | grep -q "armstrong_gazebo"; then
    echo "   ✅ Gazebo container is running"
else
    echo "   ❌ Gazebo container is NOT running"
    echo "   Run: docker-compose up -d gazebo"
    exit 1
fi

echo ""
echo "2️⃣  Checking ROS2 nodes..."
docker-compose exec -T gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    source install/setup.bash 2>/dev/null || true &&
    ros2 node list
"

echo ""
echo "3️⃣  Checking ROS2 topics..."
docker-compose exec -T gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    source install/setup.bash 2>/dev/null || true &&
    ros2 topic list
"

echo ""
echo "4️⃣  Checking if Foxglove bridge is running..."
if docker-compose exec -T gazebo bash -c "source /opt/ros/humble/setup.bash && ros2 node list" | grep -q "foxglove"; then
    echo "   ✅ Foxglove bridge is running"
else
    echo "   ⚠️  Foxglove bridge is NOT running"
    echo "   Start it with the full launch file"
fi

echo ""
echo "5️⃣  Checking if environment_visualizer is running..."
if docker-compose exec -T gazebo bash -c "source /opt/ros/humble/setup.bash && ros2 node list" | grep -q "environment_visualizer"; then
    echo "   ✅ Environment visualizer is running"
else
    echo "   ⚠️  Environment visualizer is NOT running"
    echo "   This publishes block/bin markers to Foxglove"
fi

echo ""
echo "6️⃣  Checking for /environment_markers topic..."
if docker-compose exec -T gazebo bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" | grep -q "/environment_markers"; then
    echo "   ✅ /environment_markers topic exists"
    echo ""
    echo "   📊 Checking marker data..."
    docker-compose exec -T gazebo bash -c "
        timeout 3 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic echo /environment_markers --once' 2>/dev/null
    " || echo "   (No data received in 3 seconds)"
else
    echo "   ❌ /environment_markers topic NOT found"
fi

echo ""
echo "7️⃣  Checking for Gazebo models in simulation..."
docker-compose exec -T gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 service call /gazebo/get_model_list gazebo_msgs/srv/GetModelList 2>/dev/null
" | grep -A 20 "name:" || echo "   ⚠️  Could not query Gazebo models"

echo ""
echo "8️⃣  Checking joint controllers..."
if docker-compose exec -T gazebo bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" | grep -q "/joint_states"; then
    echo "   ✅ /joint_states topic exists"
    echo "   Checking if data is being published..."
    docker-compose exec -T gazebo bash -c "
        timeout 3 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic hz /joint_states' 2>/dev/null
    " || echo "   ⚠️  No joint state data (robot might not be spawned)"
else
    echo "   ❌ No /joint_states topic"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📋 SUMMARY & RECOMMENDATIONS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "To see blocks/bins in Foxglove:"
echo "  1. Add a 3D panel in Foxglove"
echo "  2. In panel settings, subscribe to: /environment_markers"
echo "  3. Make sure 'Fixed Frame' is set to 'world'"
echo ""
echo "To make the arm move:"
echo "  1. Ensure robot is spawned and controllers are loaded"
echo "  2. Run: docker-compose exec gazebo bash"
echo "  3. Inside container:"
echo "     source /opt/ros/humble/setup.bash"
echo "     cd /workspace && source install/setup.bash"
echo "     ros2 run ur5_gazebo pick_place_controller"
echo ""
echo "For a simple test movement, try:"
echo "  ros2 topic pub /joint_states sensor_msgs/msg/JointState ..."
echo ""

