# 🦊 Foxglove Studio Setup Guide

## Overview
Foxglove Studio is a web-based robotics visualization tool that can connect to your ROS2 simulation running in Docker.

## Quick Start

### 1. Restart Your Containers
Since we've updated the configuration, you need to rebuild:

```bash
cd /Users/ryanrahman/Armstrong
docker-compose down
docker-compose up -d gazebo moveit
```

Wait for the containers to finish installing packages (this may take a few minutes on first run).

### 2. Rebuild Your Workspace
The Foxglove bridge and environment visualizer are now included in the launch file:

```bash
docker-compose exec gazebo bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /workspace &&
  colcon build --packages-select ur5_gazebo ur5_moveit_config &&
  source install/setup.bash
"
```

### 3. Launch the Simulation
Start the complete simulation with Foxglove bridge:

```bash
docker-compose exec gazebo bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /workspace &&
  source install/setup.bash &&
  ros2 launch ur5_gazebo ur5.launch.py
"
```

This will start:
- ✅ Gazebo simulation
- ✅ Robot state publisher
- ✅ Joint state publisher
- ✅ Camera detector
- ✅ **Foxglove bridge** (on port 8765)
- ✅ **Environment visualizer** (publishing markers)

### 4. Connect Foxglove Studio

#### Option A: Foxglove Studio Desktop App
1. Download from: https://foxglove.dev/download
2. Install and open Foxglove Studio
3. Click **"Open Connection"**
4. Select **"Rosbridge (ROS 1 & 2)"**
5. Enter WebSocket URL: `ws://localhost:8765`
6. Click **"Open"**

#### Option B: Foxglove Studio Web App
1. Go to: https://app.foxglove.dev
2. Click **"Open Connection"**
3. Select **"Rosbridge (ROS 1 & 2)"**
4. Enter WebSocket URL: `ws://localhost:8765`
5. Click **"Open"**

### 5. Add Visualization Panels

Once connected, add panels to visualize your data:

#### Add 3D Panel for Robot:
1. Click **"+"** to add a panel
2. Select **"3D"**
3. In the panel settings, enable:
   - `/robot_description` (Robot model)
   - `/tf` (Transforms)
   - `/environment_markers` (Environment objects)

#### Add Joint State Panel:
1. Add a new panel
2. Select **"State Transitions"** or **"Plot"**
3. Subscribe to `/joint_states`

#### Add Image Panel (if using camera):
1. Add a new panel
2. Select **"Image"**
3. Subscribe to camera topics like `/camera/image_raw`

#### Add Topic List:
1. Add a panel
2. Select **"Topic List"**
3. You should see all available topics

## Troubleshooting

### ❌ Can't connect to ws://localhost:8765

**Check if the bridge is running:**
```bash
docker-compose exec gazebo bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 node list | grep foxglove
"
```

**Check the logs:**
```bash
docker-compose logs -f gazebo
```

**Verify the port is exposed:**
```bash
docker ps | grep armstrong_gazebo
# Should show 8765:8765 in the PORTS column
```

**Manually start the bridge (if launch file doesn't work):**
```bash
docker-compose exec gazebo bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /workspace &&
  source install/setup.bash &&
  ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
"
```

### ❌ Foxglove connects but no topics appear

**Check that nodes are running:**
```bash
docker-compose exec gazebo bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 node list
"
```

**Check available topics:**
```bash
docker-compose exec gazebo bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 topic list
"
```

**Check if data is being published:**
```bash
docker-compose exec gazebo bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 topic hz /joint_states
"
```

### ❌ Robot model doesn't appear in 3D view

**Check if robot_description is published:**
```bash
docker-compose exec gazebo bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 topic echo /robot_description --once
"
```

**Check if TF is being published:**
```bash
docker-compose exec gazebo bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 run tf2_ros tf2_echo world base_link
"
```

### ❌ Environment markers don't appear

**Check if the visualizer is running:**
```bash
docker-compose exec gazebo bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 node list | grep environment_visualizer
"
```

**Check if markers are being published:**
```bash
docker-compose exec gazebo bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 topic echo /environment_markers --once
"
```

## What You Should See

When everything is working:
1. **3D View**: UR5 robot arm, blocks, bins, and environment
2. **Joint States**: Real-time joint angles and velocities
3. **TF Tree**: Transform relationships between robot links
4. **Camera Feed**: (if camera is running) Live camera view
5. **Environment Markers**: Colored cubes representing blocks and bins

## Available Topics

After launching, you should see topics like:
- `/joint_states` - Robot joint positions/velocities
- `/tf` and `/tf_static` - Transform tree
- `/robot_description` - URDF of the robot
- `/environment_markers` - Visualization markers for objects
- `/camera/image_raw` - Camera feed (if available)
- `/clock` - Simulation time

## Pro Tips

1. **Save Your Layout**: Once you configure panels, save the layout in Foxglove for future use
2. **Use Playback**: Record data with `ros2 bag record` and play it back in Foxglove
3. **Multiple Connections**: You can connect multiple Foxglove instances to the same bridge
4. **Mobile Access**: Open Foxglove web app on your phone/tablet to monitor remotely

## Alternative: ROS Bridge (if Foxglove Bridge doesn't work)

If the Foxglove bridge has issues, you can use the standard ROS bridge:

```bash
# Install rosbridge
docker-compose exec gazebo bash -c "
  apt-get update &&
  apt-get install -y ros-humble-rosbridge-server
"

# Run rosbridge
docker-compose exec gazebo bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml
"
```

Then connect Foxglove to `ws://localhost:9090`

## Need More Help?

- Foxglove Docs: https://docs.foxglove.dev
- ROS2 Humble Docs: https://docs.ros.org/en/humble/
- Check container logs: `docker-compose logs -f gazebo`
- List running nodes: `ros2 node list`
- List topics: `ros2 topic list`
- Check node info: `ros2 node info /foxglove_bridge`

