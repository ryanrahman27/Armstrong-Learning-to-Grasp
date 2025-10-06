#!/bin/bash

echo "🤖 Pick and Place Movement Sequence"
echo "===================================="
echo ""
echo "Watch in Foxglove as the robot performs a pick and place sequence!"
echo ""

# Function to send joint command
send_joints() {
    local phase=$1
    shift
    local positions="$@"
    
    echo "🎯 $phase"
    docker-compose exec -T gazebo bash -c "
        source /opt/ros/humble/setup.bash &&
        ros2 topic pub /set_joint_trajectory trajectory_msgs/msg/JointTrajectory \
        '{joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
          points: [{positions: [$positions], time_from_start: {sec: 2}}]}' --once
    " > /dev/null 2>&1
    sleep 3
}

echo "Starting demonstration..."
echo ""

# Home position
send_joints "Phase 1: Moving to home position" "0.0, -1.57, 0.0, -1.57, 0.0, 0.0"

# Reach toward block
send_joints "Phase 2: Reaching toward block" "0.3, -1.0, -1.2, -1.57, 0.0, 0.0"

# Lower to grasp
send_joints "Phase 3: Lowering to grasp block" "0.3, -0.8, -1.5, -1.8, 0.0, 0.0"

# Lift with block
send_joints "Phase 4: Lifting block" "0.3, -1.3, -1.0, -1.57, 0.0, 0.0"

# Move to bin
send_joints "Phase 5: Moving to bin" "-0.5, -1.2, -1.2, -1.57, 0.0, 0.0"

# Lower to place
send_joints "Phase 6: Placing block in bin" "-0.5, -0.9, -1.4, -1.8, 0.0, 0.0"

# Return home
send_joints "Phase 7: Returning to home" "0.0, -1.57, 0.0, -1.57, 0.0, 0.0"

echo ""
echo "✅ Pick and place sequence complete!"
echo ""
echo "The robot has completed a simulated pick and place operation!"
echo "In Foxglove, you should have seen the robot:"
echo "  1. ✅ Move to home position"
echo "  2. ✅ Reach toward the block"
echo "  3. ✅ Lower to grasp height"
echo "  4. ✅ Lift up (simulating picking)"
echo "  5. ✅ Move to the bin location"
echo "  6. ✅ Lower to place"
echo "  7. ✅ Return home"
echo ""
echo "🎉 Demo complete!"

