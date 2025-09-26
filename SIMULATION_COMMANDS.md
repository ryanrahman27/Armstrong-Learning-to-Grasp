# 🎮 Simulation Commands (You're Inside the Container!)

## **You're now in a Linux container with ROS2 + Gazebo! 🤖**

### **First, set up the environment:**
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Check ROS2 is working
ros2 --help

# Check what packages are available
ros2 pkg list | grep gazebo
```

### **Install additional packages needed for simulation:**
```bash
# Update package list
apt update

# Install Gazebo and robotics packages
apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-ur-description \
    ros-humble-moveit \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro
```

### **Start Gazebo simulation:**
```bash
# Launch Gazebo (headless mode)
ros2 launch gazebo_ros gazebo.launch.py

# Or in a new terminal, start with GUI (if X11 works)
gazebo
```

### **Work with your Armstrong project:**
```bash
# Go to your workspace
cd /workspace

# Build your packages
colcon build --packages-select ur5_gazebo ur5_moveit_config

# Source your workspace
source install/setup.bash

# List your custom packages
ros2 pkg list | grep ur5
```

### **Run your robotics nodes:**
```bash
# Camera detector
python3 src/ur5_gazebo/scripts/camera_detector.py &

# Pick and place controller
python3 src/ur5_gazebo/scripts/pick_place_controller.py &

# Monitor the system
ros2 topic list
ros2 node list
```

### **Load robot models:**
```bash
# Load UR5 robot description
ros2 param set /robot_state_publisher robot_description "$(xacro src/ur5_gazebo/urdf/ur5.urdf.xacro)"

# Start robot state publisher
ros2 run robot_state_publisher robot_state_publisher &

# Start joint state publisher
ros2 run joint_state_publisher joint_state_publisher &
```

### **Spawn robot in Gazebo:**
```bash
# Spawn the UR5 robot
ros2 run gazebo_ros spawn_entity.py \
    -file /workspace/src/ur5_gazebo/urdf/ur5.urdf.xacro \
    -entity ur5 \
    -x 0 -y 0 -z 0
```

## **🎯 Quick Start Sequence:**

1. **Install packages:** `apt update && apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros`
2. **Start Gazebo:** `ros2 launch gazebo_ros gazebo.launch.py`
3. **In new terminal:** Access container again and load your robot
4. **Monitor:** `ros2 topic list` and `ros2 node list`

## **🔧 Multiple Terminal Sessions:**

To work with multiple terminals in the container:
```bash
# In your macOS terminal, run:
docker exec -it armstrong_test bash
```

This gives you additional terminal sessions in the same container!

**You now have full ROS2 + Gazebo simulation capabilities! 🚀**