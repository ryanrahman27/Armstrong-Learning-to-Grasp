# 🦊 Complete Foxglove Visualization Guide

## Summary of Changes Made

I've fixed your Foxglove setup so you can see blocks, bins, and robot movement. Here's what was done:

### ✅ Fixed Issues:
1. **Foxglove Bridge** - Installed and configured to run automatically
2. **Environment Visualizer** - Set up to publish block/bin markers  
3. **Robot Joint Controllers** - Added Gazebo plugins for movement
4. **Launch File** - Updated to start all visualization components

---

## 🚀 Quick Start (Restart Your Simulation)

### Step 1: Rebuild with New Configuration

```bash
cd /Users/ryanrahman/Armstrong

# Stop current simulation
docker-compose down

# Restart with Foxglove support
./start_with_foxglove.sh
```

This will:
- Install `ros-humble-foxglove-bridge`
- Build your workspace with updated URDF
- Launch Gazebo, Foxglove bridge, and environment visualizer
- Expose port 8765 for Foxglove connection

### Step 2: Connect Foxglove

1. **Open Foxglove Studio** (download from https://foxglove.dev/download)
   - OR use web version: https://app.foxglove.dev

2. Click **"Open Connection"**

3. Select **"Rosbridge (ROS 1 & 2)"**

4. Enter: `ws://localhost:8765`

5. Click **"Open"**

You should see: "Connected to ws://localhost:8765" ✅

### Step 3: Add Visualization Panels

#### Add 3D Panel:
1. Click **"+"** button → Select **"3D"**
2. In 3D panel settings (gear icon):
   - Set **"Fixed Frame"** to `world`
   - Under "Topics", enable:
     - `/tf` (transforms)
     - `/robot_description` (robot model) 
     - `/environment_markers` (blocks and bins)

#### You Should Now See:
- 🤖 **UR5 Robot Arm** (gray, 6-DOF manipulator)
- 🔴 **Red Block** at position (0.5, 0.3, 0.05)
- ⬜ **Gray Bin** at position (0.7, 0.0, 0.05)
- ⬛ **Ground Plane** (dark gray reference)

---

## 🎮 Making the Robot Move

The robot is now controllable via Gazebo! Here are your options:

### Option A: Test Basic Movement (Simplest)

```bash
./test_robot_movement.sh
```

This sends force commands to move the shoulder joint. Watch in Foxglove!

### Option B: Manual Joint Commands

Enter the container and send commands directly:

```bash
docker-compose exec gazebo bash

# Inside container:
source /opt/ros/humble/setup.bash

# Move shoulder pan joint
ros2 service call /gazebo/apply_joint_effort gazebo_msgs/srv/ApplyJointEffort "{
    joint_name: 'shoulder_pan_joint',
    effort: 50.0,
    duration: {sec: 3}
}"
```

### Option C: Continuous Demo Movement

Run the simple arm mover script:

```bash
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    python3 simple_arm_mover.py
"
```

This creates a continuous demo:
1. Move to home position
2. Reach forward
3. Move to side
4. Wave continuously

**Note**: This requires the trajectory controller to be working. If it doesn't work, use Option A or B.

### Option D: Full Pick-and-Place (Advanced)

For the complete demonstration with MoveIt:

**Terminal 1 - Start MoveIt:**
```bash
docker-compose exec moveit bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    source install/setup.bash &&
    ros2 launch ur5_moveit_config move_group.launch.py
"
```

**Terminal 2 - Start Camera:**
```bash
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    source install/setup.bash &&
    ros2 run ur5_gazebo camera_detector
"
```

**Terminal 3 - Start Controller:**
```bash
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    source install/setup.bash &&
    ros2 run ur5_gazebo pick_place_controller
"
```

The robot will autonomously:
1. Detect the red block
2. Plan a trajectory to reach it
3. Pick it up
4. Move to the bin
5. Place it down

---

## 🔍 Diagnostics

If something isn't working, run:

```bash
./diagnose_simulation.sh
```

This checks:
- ✅ Container status
- ✅ Running nodes
- ✅ Available topics
- ✅ Foxglove bridge status
- ✅ Environment markers
- ✅ Joint states

---

## 📊 Foxglove Panel Configuration Tips

### Recommended Panels:

1. **3D Panel** (Primary view)
   - Shows robot, blocks, bins, transforms
   - Fixed frame: `world`
   - Enable: `/tf`, `/robot_description`, `/environment_markers`

2. **Topic List Panel**
   - See all available topics
   - Check data rates

3. **Plot Panel**
   - Plot: `/joint_states/position[0]` to see joint angles over time
   - Add multiple lines for different joints

4. **State Transitions Panel**
   - Visualize `/joint_states` as a timeline

5. **Image Panel** (if camera active)
   - Subscribe to `/camera/image_raw`
   - See robot's camera view

### Save Your Layout!
Once configured, click "Layout" → "Save" to persist your setup.

---

## 🐛 Troubleshooting

### ❌ "Can't see blocks/bins in 3D view"

**Solution**: Add `/environment_markers` topic
1. Click gear icon on 3D panel
2. Scroll to "Topics" section  
3. Find `/environment_markers` and enable it
4. Verify "Fixed Frame" is set to `world`

**Alternative**: Check if visualizer is running:
```bash
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 node list | grep environment_visualizer
"
```

If not found, restart with `./start_with_foxglove.sh`

### ❌ "Robot doesn't move when I send commands"

**Check joint states are being published:**
```bash
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 topic hz /joint_states
"
```

If no data, the robot might not be spawned. Check:
```bash
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 service call /gazebo/get_model_list gazebo_msgs/srv/GetModelList
"
```

Should show `ur5` in the model list.

### ❌ "Foxglove shows 'Disconnected'"

**Check bridge is running:**
```bash
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 node list | grep foxglove
"
```

**Check port is exposed:**
```bash
docker ps | grep armstrong_gazebo
# Should show: 0.0.0.0:8765->8765/tcp
```

**Restart bridge manually:**
```bash
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
"
```

### ❌ "Robot model doesn't appear"

**Check robot_description topic:**
```bash
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 topic list | grep robot_description
"
```

**Echo to verify URDF is published:**
```bash
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 topic echo /robot_description --once
"
```

---

## 📋 Available Topics (What You Should See)

After launching, these topics should be available:

### Robot State:
- `/joint_states` - Current joint positions/velocities
- `/robot_description` - URDF model
- `/tf` - Transform tree (robot kinematics)
- `/tf_static` - Static transforms

### Visualization:
- `/environment_markers` - Blocks, bins, ground plane
- `/camera/image_raw` - Camera feed (if active)

### Control:
- `/set_joint_trajectory` - Command robot motion
- `/gazebo/apply_joint_effort` - Apply forces to joints
- `/move_group/*` - MoveIt action topics

### System:
- `/clock` - Simulation time
- `/rosout` - System logs

---

## 🎯 Next Steps

### Once Everything is Working:

1. **Record Data**: Use rosbag to record demonstrations
   ```bash
   ros2 bag record -a
   ```

2. **Train Behavioral Cloning**: Use recorded data to train a policy
   ```bash
   docker-compose up bc_training
   ```

3. **Compare Methods**: Run the comparison evaluator
   ```bash
   ros2 run ur5_gazebo comparison_evaluator
   ```

4. **Visualize Results**: Plot trajectories and success rates

---

## 📚 Reference Files

- **FOXGLOVE_SETUP.md** - Detailed Foxglove installation guide
- **FOXGLOVE_QUICK_FIX.md** - Quick troubleshooting steps
- **diagnose_simulation.sh** - Diagnostic script
- **test_robot_movement.sh** - Test robot motion
- **start_with_foxglove.sh** - Complete startup script

---

## 💡 Pro Tips

1. **Adjust Camera in Foxglove**: Use mouse to orbit, pan, zoom in 3D view
2. **Multiple Connections**: Open Foxglove on multiple devices simultaneously
3. **Mobile Access**: Connect from phone/tablet browser to monitor remotely
4. **Save Layouts**: Create different layouts for different tasks
5. **Use Playback**: Record bags, then analyze in Foxglove with timeline scrubbing

---

## 🎉 You're All Set!

Your Armstrong robotics platform now has full Foxglove visualization support! You can:

- ✅ See the robot in real-time
- ✅ Visualize blocks and bins
- ✅ Monitor joint states and transforms  
- ✅ Watch camera feeds
- ✅ Send movement commands
- ✅ Debug with topic inspection
- ✅ Record and playback demonstrations

Happy robot programming! 🤖

