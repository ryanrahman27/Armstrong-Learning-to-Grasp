#!/bin/bash

echo "🤖 Starting Pick and Place Demo"
echo "================================"
echo ""
echo "This will launch the autonomous pick and place demonstration!"
echo ""

# Check if simulation is running
if ! docker ps | grep -q "armstrong_gazebo"; then
    echo "❌ Gazebo container is not running!"
    echo "   Please start it first with: ./start_with_foxglove.sh"
    exit 1
fi

echo "✅ Gazebo is running"
echo ""

# Start MoveIt in moveit container (in background)
echo "🧠 Starting MoveIt motion planner..."
docker-compose exec -d moveit bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    source install/setup.bash &&
    ros2 launch ur5_moveit_config moveit.launch.py
"

echo "⏳ Waiting for MoveIt to initialize (15 seconds)..."
sleep 15

echo ""
echo "✅ MoveIt should be running now"
echo ""

# Check if MoveIt is running
if docker-compose exec -T moveit bash -c "source /opt/ros/humble/setup.bash && ros2 node list" 2>/dev/null | grep -q "move_group"; then
    echo "✅ MoveIt move_group is running!"
else
    echo "⚠️  MoveIt may still be starting..."
fi

echo ""
echo "🎯 Starting Pick and Place Controller..."
docker-compose exec -d gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    source install/setup.bash &&
    ros2 run ur5_gazebo pick_place_controller
"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🎉 PICK AND PLACE DEMO IS STARTING!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "👀 Watch in Foxglove Studio to see:"
echo "   1. Robot detecting the red block"
echo "   2. Planning trajectory to pick position"
echo "   3. Moving to the block"
echo "   4. Grasping it"
echo "   5. Lifting and moving to bin"
echo "   6. Placing it down"
echo "   7. Returning to home position"
echo ""
echo "📋 To see logs from pick_place_controller:"
echo "   docker-compose logs -f gazebo | grep pick_place"
echo ""
echo "📋 To see MoveIt logs:"
echo "   docker-compose logs -f moveit"
echo ""
echo "🛑 To stop the demo:"
echo "   docker-compose down"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

