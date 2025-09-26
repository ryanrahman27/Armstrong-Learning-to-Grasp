# 🚀 Armstrong Working Guide

## **✅ Current Status: SYSTEM IS WORKING!**

Your robotics simulation is operational. Here's how to use it:

## **🎯 Quick Start Commands:**

### **1. Start the System:**
```bash
docker-compose up -d gazebo moveit
```

### **2. Enter Development Environment:**
```bash
docker-compose exec gazebo bash
source /opt/ros/humble/setup.bash
cd /workspace && source install/setup.bash
```

### **3. Available Scripts (Working!):**
```bash
# Camera detector (computer vision)
python3 src/ur5_gazebo/scripts/camera_detector.py &

# Pick and place controller (motion planning)
python3 src/ur5_gazebo/scripts/pick_place_controller.py &

# Demo recorder (for behavioral cloning)
python3 src/ur5_gazebo/scripts/demo_recorder.py &

# BC policy controller (imitation learning)
python3 src/ur5_gazebo/scripts/bc_policy_controller.py &

# Comparison evaluator (research metrics)
python3 src/ur5_gazebo/scripts/comparison_evaluator.py &
```

### **4. Monitor the System:**
```bash
# See active nodes
ros2 node list

# See available topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /block_pose
ros2 topic echo /joint_states
ros2 topic echo /camera/image_raw
```

## **🔬 Research Workflows:**

### **A. Computer Vision Development:**
```bash
# Edit camera detection
vim src/ur5_gazebo/scripts/camera_detector.py

# Test changes
python3 src/ur5_gazebo/scripts/camera_detector.py

# Monitor output
ros2 topic echo /block_pose
```

### **B. Motion Planning Experiments:**
```bash
# Modify planning algorithms
vim src/ur5_gazebo/scripts/pick_place_controller.py

# Test different planners
python3 src/ur5_gazebo/scripts/pick_place_controller.py

# Monitor trajectories
ros2 topic echo /joint_states
```

### **C. Behavioral Cloning Research:**
```bash
# Record demonstrations
python3 src/ur5_gazebo/scripts/demo_recorder.py &
ros2 topic pub /record_demo std_msgs/msg/Bool "data: true"

# Train models (in separate terminal)
docker-compose run bc_training python3 /workspace/src/bc_training/train_policy.py \
    --bag_path /data/demonstrations/demo_* \
    --model_path /data/models/bc_model.h5

# Test trained policy
python3 src/ur5_gazebo/scripts/bc_policy_controller.py
```

### **D. Performance Comparison:**
```bash
# Run automated comparison
python3 src/ur5_gazebo/scripts/comparison_evaluator.py &

# Start evaluations
ros2 topic pub /start_evaluation std_msgs/msg/String "data: 'moveit'"
ros2 topic pub /start_evaluation std_msgs/msg/String "data: 'bc'"

# Monitor results
ros2 topic echo /evaluation_status
```

## **📊 Data Collection:**

### **Save Experimental Data:**
```bash
# Record topics to bag files
ros2 bag record /joint_states /block_pose /camera/image_raw

# Export topic data
ros2 topic echo /joint_states > joint_trajectory.txt
```

### **Access Results:**
```bash
# View result files
ls -la /data/
cat /data/evaluation_*.json
```

## **🛠️ Development Tips:**

### **Edit Code Live:**
```bash
# Make changes to scripts in workspace/src/
# Restart nodes to see changes
pkill -f camera_detector
python3 src/ur5_gazebo/scripts/camera_detector.py &
```

### **Debug Issues:**
```bash
# Check ROS2 environment
env | grep ROS
ros2 doctor

# Verify packages
ros2 pkg list | grep ur5
```

### **Multiple Terminal Sessions:**
```bash
# Terminal 1: Node management
docker-compose exec gazebo bash

# Terminal 2: Data monitoring  
docker-compose exec gazebo bash

# Terminal 3: Development
docker-compose exec gazebo bash
```

## **🎯 Research Capabilities Available NOW:**

✅ **6-DOF robot simulation** (UR5 arm)  
✅ **Computer vision** (block detection)  
✅ **Motion planning** (MoveIt2 integration)  
✅ **Behavioral cloning** (TensorFlow training)  
✅ **Data collection** (ROS bag recording)  
✅ **Performance metrics** (automated evaluation)  
✅ **Algorithm comparison** (IK vs BC)  

## **🔄 Typical Workflow:**

```bash
# 1. Start system
docker-compose up -d gazebo moveit

# 2. Enter container
docker-compose exec gazebo bash
source /opt/ros/humble/setup.bash
cd /workspace && source install/setup.bash

# 3. Start core nodes
python3 src/ur5_gazebo/scripts/camera_detector.py &
python3 src/ur5_gazebo/scripts/pick_place_controller.py &

# 4. Monitor and experiment
ros2 topic list
ros2 topic echo /block_pose

# 5. Collect data and analyze
ros2 bag record /joint_states /block_pose
```

**You now have a fully functional robotics research platform! 🤖**

## **Next: Choose Your Research Direction:**
- 🎯 **Motion Planning**: Experiment with different OMPL algorithms
- 🤖 **Computer Vision**: Improve block detection accuracy  
- 🧠 **Machine Learning**: Train better behavioral cloning policies
- 📊 **Analysis**: Compare traditional vs learning approaches