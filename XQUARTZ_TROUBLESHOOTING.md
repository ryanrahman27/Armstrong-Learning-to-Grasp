# 🖥️ XQuartz + Docker Troubleshooting

## **Current Issue:** 
RViz still can't connect to XQuartz display despite proper setup.

## **Why This Happens:**
Docker Desktop on macOS has known issues with X11 forwarding, especially with:
- Apple Silicon Macs (ARM64 vs x86_64 container mismatch)
- Recent macOS security changes
- Docker networking complications

## **✅ Working Solutions:**

### **Option 1: Use Native ROS2 (Recommended)**
```bash
# Install ROS2 natively on macOS
brew tap ros/robot_localization  
brew install ros-humble-desktop-full
brew install ros-humble-moveit
brew install ros-humble-rviz2

# Start only simulation containers
docker-compose up -d gazebo moveit

# Run native RViz (will work perfectly)
source /opt/homebrew/setup.bash
ros2 launch ur5_moveit_config rviz.launch.py
```

### **Option 2: Advanced XQuartz Setup**
```bash
# 1. Ensure XQuartz is fully configured
# In XQuartz Preferences → Security:
#   ✅ "Allow connections from network clients"
#   ✅ "Authenticate connections" (uncheck this)

# 2. Configure XQuartz for Docker
echo "export DISPLAY=:0" >> ~/.zshrc
source ~/.zshrc
xhost +local:docker
xhost +localhost
xhost +127.0.0.1

# 3. Try alternative display configurations
export DISPLAY=host.docker.internal:0
# OR
export DISPLAY=docker.for.mac.localhost:0
```

### **Option 3: VNC-based Visualization**
Create a VNC server in the container:
```dockerfile
# Add to a custom Dockerfile
RUN apt-get update && apt-get install -y \
    x11vnc \
    xvfb \
    fluxbox \
    websockify

# Run with VNC
CMD x11vnc -forever -usepw -create
```

### **Option 4: Web-based RViz**
Use web-based visualization tools:
```bash
# Install ros2-web-bridge
pip install ros2-web-bridge

# Use browser-based visualization
# Access via http://localhost:8080
```

## **🔧 Current Workaround:**

Since GUI is challenging, you can still use the full system:

### **Headless Development:**
```bash
# 1. Start core system
docker-compose up -d gazebo moveit

# 2. Access containers for development
docker-compose exec gazebo bash
source /opt/ros/humble/setup.bash
cd /workspace && source install/setup.bash

# 3. Run all components manually
ros2 run ur5_gazebo camera_detector &
ros2 run ur5_gazebo pick_place_controller &
ros2 run ur5_gazebo demo_recorder &

# 4. Monitor with command line
ros2 node list
ros2 topic list
ros2 topic echo /block_pose
```

### **Automated Experiments:**
```bash
# Run pick-and-place experiments programmatically
docker-compose exec gazebo bash -c "
  source /opt/ros/humble/setup.bash && 
  cd /workspace && source install/setup.bash && 
  ros2 run ur5_gazebo comparison_evaluator
"
```

## **🎯 Recommended Path:**

**For immediate use:** Use the headless development approach above  
**For full visualization:** Install native ROS2 on macOS  
**For convenience:** Continue with headless until you need visualization  

The core robotics functionality (simulation, planning, learning) works perfectly without GUI! 🤖