# 🦊 Quick Fix: See Blocks and Move Robot in Foxglove

## Issue 1: Can't see blocks and bins in Foxglove

### Solution: Add the markers topic to your 3D panel

1. **In Foxglove Studio**, click on your 3D panel settings (gear icon)
2. Scroll down to find **"Topics"** section
3. Look for and enable `/environment_markers`
4. Make sure **"Fixed Frame"** at the top is set to `world`
5. You should now see:
   - 🔴 Red block at position (0.5, 0.3, 0.05)
   - ⬜ Gray bin at position (0.7, 0.0, 0.05)
   - Gray ground plane

### Alternative: Check if visualizer is running

Run diagnostics:
```bash
./diagnose_simulation.sh
```

If environment_visualizer is not running, restart the simulation:
```bash
docker-compose down
./start_with_foxglove.sh
```

---

## Issue 2: Robot arm doesn't move

The robot arm is static because:
1. No joint controllers are loaded yet
2. No motion commands are being sent

### Quick Solution A: Use Simple Arm Mover (No MoveIt needed)

```bash
# In a new terminal
docker-compose exec gazebo bash

# Inside the container:
source /opt/ros/humble/setup.bash
cd /workspace
python3 simple_arm_mover.py
```

This will make the arm:
- Go to home position
- Reach forward
- Move to the side
- Wave continuously

**Note**: This requires ros2_control to be set up. If it doesn't work, see Solution B.

### Solution B: Manually command joint positions

```bash
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 topic pub /joint_states sensor_msgs/msg/JointState '{
        name: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
        position: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    }' --once
"
```

### Solution C: Check if robot controllers need to be loaded

The UR5 robot may need joint trajectory controllers. Check what's available:

```bash
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 control list_controllers
"
```

If no controllers are loaded, you may need to load them:

```bash
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    ros2 run controller_manager spawner joint_state_broadcaster &&
    ros2 run controller_manager spawner joint_trajectory_controller
"
```

---

## Full System Check

Run the diagnostic script to see what's working:

```bash
./diagnose_simulation.sh
```

This will tell you:
- ✅ What nodes are running
- ✅ What topics are available
- ✅ If Foxglove bridge is active
- ✅ If environment markers are being published
- ✅ If joint states are available

---

## What You Should See in Foxglove

### In the 3D Panel:
- **Robot arm** (UR5 with 6 joints)
- **Red block** (small cube)
- **Gray bin** (larger box)
- **Ground plane** (reference grid)
- **TF frames** (coordinate axes for each joint)

### In Topic List Panel:
- `/joint_states` - Robot joint positions/velocities
- `/tf` and `/tf_static` - Transform tree
- `/robot_description` - URDF model
- `/environment_markers` - Visualization markers for blocks/bins
- `/camera/image_raw` - Camera view (if camera is active)

---

## Common Issues

### "I see the robot but it's frozen"
- The robot needs controllers to move
- Try the simple_arm_mover.py script above
- Or check if ros2_control is properly configured in the URDF

### "I see markers but they don't match Gazebo"
- The environment_visualizer publishes markers at fixed positions
- These show where objects SHOULD be in the Gazebo world
- If Gazebo doesn't have the objects, the world file might not be loaded

### "Markers appear but are tiny/huge"
- Check the "Fixed Frame" in Foxglove 3D panel settings
- Should be set to `world` or `base_link`
- Adjust camera position by dragging in the 3D view

### "Robot moves but blocks don't"
- Blocks in Gazebo are physics objects
- You need a gripper or collision contact to move them
- The simple_arm_mover doesn't interact with objects
- For pick-and-place, use the full MoveIt controller

---

## Next Steps: Full Pick-and-Place Demo

Once you have basic visualization working, try the full pick-and-place:

```bash
# Terminal 1: Start MoveIt
docker-compose exec moveit bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    source install/setup.bash &&
    ros2 launch ur5_moveit_config move_group.launch.py
"

# Terminal 2: Start camera detector
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    source install/setup.bash &&
    ros2 run ur5_gazebo camera_detector
"

# Terminal 3: Start pick-place controller
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    source install/setup.bash &&
    ros2 run ur5_gazebo pick_place_controller
"
```

This will make the robot:
1. Detect the red block with the camera
2. Plan a motion to pick it up
3. Lift it
4. Move to the bin
5. Place it down
6. Return to home position

---

## Still Having Issues?

1. Check logs: `docker-compose logs -f gazebo`
2. List nodes: `docker-compose exec gazebo bash -c "source /opt/ros/humble/setup.bash && ros2 node list"`
3. List topics: `docker-compose exec gazebo bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"`
4. Check if Gazebo is actually running: `docker ps | grep armstrong`

For more detailed troubleshooting, see `FOXGLOVE_SETUP.md`

