#!/bin/bash

echo "🔍 Checking Environment Markers Status"
echo "======================================"
echo ""

# Check if environment_visualizer node is running
echo "1️⃣  Checking if environment_visualizer node is running..."
if docker-compose exec -T gazebo bash -c "source /opt/ros/humble/setup.bash && ros2 node list" 2>/dev/null | grep -q "environment_visualizer"; then
    echo "   ✅ environment_visualizer node is RUNNING"
else
    echo "   ❌ environment_visualizer node is NOT running"
    echo "   Start it with: ros2 launch ur5_gazebo ur5.launch.py"
fi

echo ""

# Check if /environment_markers topic exists
echo "2️⃣  Checking if /environment_markers topic exists..."
if docker-compose exec -T gazebo bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" 2>/dev/null | grep -q "/environment_markers"; then
    echo "   ✅ /environment_markers topic EXISTS"
    
    # Check publish rate
    echo ""
    echo "3️⃣  Checking marker publish rate..."
    docker-compose exec -T gazebo bash -c "
        timeout 5 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic hz /environment_markers' 2>/dev/null
    " || echo "   ⚠️  Could not measure rate (may need more time)"
    
else
    echo "   ❌ /environment_markers topic does NOT exist"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "💡 Summary"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "The markers will automatically run when you start the simulation with:"
echo "  ros2 launch ur5_gazebo ur5.launch.py"
echo ""
echo "They publish at 1 Hz (once per second) to /environment_markers"
echo ""
echo "In Foxglove, make sure /environment_markers is enabled in your 3D panel!"
echo ""

