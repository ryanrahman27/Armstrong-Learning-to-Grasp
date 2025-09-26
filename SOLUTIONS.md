# 🔧 Armstrong Solutions Guide

## **Current Status:** ✅ Core system working, GUI needs setup

### **For RViz Visualization (Pick One):**

#### **Option 1: Install XQuartz (Docker GUI)**
```bash
# Install XQuartz
brew install --cask xquartz

# Configure XQuartz
open -a XQuartz
# In XQuartz Preferences → Security → Check "Allow connections from network clients"
# Restart XQuartz

# Set display
export DISPLAY=:0

# Run RViz
docker-compose up rviz
```

#### **Option 2: Use Native ROS2 (Recommended)**
```bash
# Install ROS2 on macOS
brew install ros-humble-desktop

# Start simulation containers only
docker-compose up -d gazebo moveit

# Run native RViz (after installing additional packages)
source /opt/homebrew/setup.bash
ros2 launch ur5_moveit_config rviz.launch.py
```

#### **Option 3: Web-based Visualization**
```bash
# Add to docker-compose.yml:
# - NoVNC or web-based RViz
# - Access via http://localhost:6080
```

### **For Core Development (Working Now):**

#### **Headless Mode:**
```bash
# Start simulation and planning
docker-compose up -d gazebo moveit

# View logs
docker-compose logs -f gazebo
docker-compose logs -f moveit

# Access containers
docker-compose exec gazebo bash
# Inside container: source /opt/ros/humble/setup.bash
```

#### **Manual Testing:**
```bash
# Inside gazebo container
source /opt/ros/humble/setup.bash
cd /workspace && source install/setup.bash

# Run individual components
ros2 run ur5_gazebo camera_detector
ros2 run ur5_gazebo pick_place_controller
ros2 topic list
ros2 node list
```

### **For Full Experiment:**

#### **Automated (when GUI is set up):**
```bash
./run_experiment.sh
```

#### **Manual Steps:**
```bash
# 1. Start containers
docker-compose up -d gazebo moveit

# 2. Wait for startup
sleep 30

# 3. Run individual nodes inside containers
docker-compose exec gazebo bash -c "source /opt/ros/humble/setup.bash && cd /workspace && source install/setup.bash && ros2 run ur5_gazebo camera_detector"

# 4. In separate terminals, run other components
docker-compose exec gazebo bash -c "source /opt/ros/humble/setup.bash && cd /workspace && source install/setup.bash && ros2 run ur5_gazebo pick_place_controller"
```

## **Development Workflow:**

### **Code Changes:**
```bash
# 1. Edit files in workspace/src/
# 2. Rebuild in container
docker-compose exec gazebo bash -c "cd /workspace && colcon build"
# 3. Source and test
docker-compose exec gazebo bash -c "cd /workspace && source install/setup.bash && ros2 run ur5_gazebo your_node"
```

### **Debugging:**
```bash
# View all logs
docker-compose logs

# Enter container for debugging
docker-compose exec gazebo bash
source /opt/ros/humble/setup.bash
cd /workspace && source install/setup.bash
ros2 node list
ros2 topic list
```

## **Quick Reference:**

### **Working Commands:**
- ✅ `./test_basic.sh` - Basic functionality test
- ✅ `docker-compose up -d gazebo moveit` - Start core system
- ✅ `docker-compose logs -f gazebo` - View simulation logs
- ✅ `docker-compose exec gazebo bash` - Enter container

### **Broken/Needs Setup:**
- ❌ `docker-compose up rviz` - Needs XQuartz or native ROS2
- ❌ `./run_experiment.sh` - Needs GUI for full automation
- ⚠️ `docker-compose exec gazebo ros2 ...` - Needs environment sourcing

### **Next Steps:**
1. **For GUI**: Install XQuartz OR native ROS2
2. **For development**: Use headless mode with container access
3. **For full experiments**: Set up visualization first

---

**The core robotics simulation is working! 🎉**  
**Choose your visualization method to unlock the full experience.**