#!/bin/bash

echo "🤖 Starting Headless Gazebo Simulation"
echo "======================================"

# Update docker-compose to run without GUI launch files
cat > docker-compose-sim.yml << EOF
services:
  gazebo_sim:
    image: osrf/ros:humble-desktop-full
    platform: linux/amd64
    container_name: armstrong_gazebo_sim
    environment:
      - DISPLAY=\${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./workspace:/workspace
      - ./data:/data
    network_mode: host
    command: >
      bash -c "
        apt-get update && apt-get install -y 
          ros-humble-gazebo-ros-pkgs 
          ros-humble-gazebo-ros 
          ros-humble-ur-description 
          ros-humble-moveit 
          ros-humble-moveit-msgs 
          ros-humble-moveit-ros-planning-interface 
          ros-humble-joint-state-publisher 
          ros-humble-robot-state-publisher 
          ros-humble-xacro 
          ros-humble-foxglove-bridge &&
        source /opt/ros/humble/setup.bash &&
        cd /workspace &&
        colcon build --packages-select ur5_gazebo ur5_moveit_config &&
        source install/setup.bash &&
        echo 'Container ready for simulation!' &&
        tail -f /dev/null
      "
EOF

echo "🚀 Starting simulation container..."
docker-compose -f docker-compose-sim.yml up -d

echo "⏳ Waiting for setup to complete..."
sleep 30

echo "✅ Container ready! Access with:"
echo "docker-compose -f docker-compose-sim.yml exec gazebo_sim bash"
echo ""
echo "Then run:"
echo "source /opt/ros/humble/setup.bash"
echo "cd /workspace && source install/setup.bash"
echo "ros2 launch gazebo_ros gazebo.launch.py"