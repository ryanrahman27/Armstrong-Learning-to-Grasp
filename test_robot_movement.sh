#!/bin/bash

echo "🤖 Testing UR5 Robot Movement"
echo "=============================="
echo ""

# Check if container is running
if ! docker ps | grep -q "armstrong_gazebo"; then
    echo "❌ Gazebo container is NOT running"
    echo "   Run: docker-compose up -d gazebo"
    exit 1
fi

echo "✅ Gazebo container is running"
echo ""
echo "🔍 Checking available Gazebo services..."
docker-compose exec -T gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 service list | grep gazebo
"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🎮 Testing Joint Movement Commands"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

echo "1️⃣  Moving shoulder_pan_joint to 0.5 rad..."
docker-compose exec -T gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 service call /gazebo/set_joint_properties gazebo_msgs/srv/SetJointProperties \"{
        joint_name: 'shoulder_pan_joint',
        ode_joint_config: {
            damping: [1.0],
            hiStop: [3.14],
            loStop: [-3.14],
            erp: [0.2],
            cfm: [0.0],
            stop_erp: [0.2],
            stop_cfm: [0.0],
            fudge_factor: [1.0],
            fmax: [100.0],
            vel: [3.15]
        }
    }\"
" 2>/dev/null

sleep 1

echo ""
echo "2️⃣  Applying force to move joint..."
docker-compose exec -T gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 service call /gazebo/apply_joint_effort gazebo_msgs/srv/ApplyJointEffort \"{
        joint_name: 'shoulder_pan_joint',
        effort: 50.0,
        start_time: {sec: 0, nanosec: 0},
        duration: {sec: 2, nanosec: 0}
    }\"
" 2>/dev/null

echo ""
echo "✅ Movement command sent!"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "💡 Watch in Foxglove to see the robot move!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "To see continuous movement, you can run:"
echo ""
echo "docker-compose exec gazebo bash"
echo "# Then inside container:"
echo "source /opt/ros/humble/setup.bash"
echo "cd /workspace && source install/setup.bash"
echo ""
echo "# Publish joint trajectory:"
echo "ros2 topic pub /set_joint_trajectory trajectory_msgs/msg/JointTrajectory \\"
echo "  '{joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],"
echo "    points: [{positions: [0.5, -1.0, -1.5, -1.57, 0.0, 0.0], time_from_start: {sec: 2}}]}' --once"
echo ""

