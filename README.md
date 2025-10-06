# Armstrong: 6-DOF Pick and Place Robotic Arm Simulation

A Docker-based ROS2 robotic arm simulation with real-time visualization using Foxglove Studio.

## Features

- **UR5 6-DOF Robotic Arm** - Complete simulation in Gazebo with physics
- **Real-time Visualization** - Foxglove Studio web-based visualization
- **Pick and Place Demo** - Automated motion sequences
- **Environment Markers** - Visual indicators for blocks and bins
- **Docker-based Setup** - Easy deployment with all dependencies included

## Quick Start

### Prerequisites

- Docker Desktop
- Foxglove Studio (desktop app or web version)

### 1. Start the Simulation

```bash
./start_with_foxglove.sh
```

This script will:
- Start Docker containers with ROS2 and Gazebo
- Install all required packages
- Build the workspace
- Launch the simulation with Foxglove bridge
- Start environment visualization

### 2. Connect Foxglove Studio

**Desktop App** (recommended):
1. Download from https://foxglove.dev/download
2. Open Foxglove Studio
3. Click "Open Connection"
4. Select "Rosbridge (ROS 1 & 2)"
5. Enter: `ws://localhost:8765`
6. Click "Open"

**Web Version**:
1. Go to https://app.foxglove.dev
2. Follow same connection steps as above

### 3. Configure Visualization

In Foxglove, add a **3D Panel** and enable:
- `/tf` - Transform tree
- `/robot_description` - Robot model
- `/environment_markers` - Blocks and bins
- Set **Fixed Frame** to `world`

### 4. Run Pick-and-Place Demo

```bash
./demo_pick_place_sequence.sh
```

Watch in Foxglove as the robot performs a 7-phase pick-and-place sequence:
1. Move to home position
2. Reach toward block
3. Lower to grasp
4. Lift block
5. Move to bin
6. Place block
7. Return home

## Project Structure

```
Armstrong/
├── docker-compose.yml                 # Multi-container configuration
├── start_with_foxglove.sh            # Main startup script
├── demo_pick_place_sequence.sh       # Pick-and-place demonstration
├── diagnose_simulation.sh            # Diagnostic tool
├── FOXGLOVE_COMPLETE_GUIDE.md        # Complete setup and troubleshooting guide
└── workspace/
    └── src/
        ├── ur5_gazebo/               # Gazebo simulation package
        │   ├── urdf/                 # Robot model with Gazebo plugins
        │   ├── worlds/               # Simulation environments
        │   ├── launch/               # Launch files
        │   └── ur5_gazebo/           # Python nodes
        │       ├── environment_visualizer.py  # Publishes markers
        │       ├── camera_detector.py         # Block detection
        │       ├── pick_place_controller.py   # Motion control
        │       └── ...
        ├── ur5_moveit_config/        # MoveIt2 configuration
        └── bc_training/              # Behavioral cloning (future)
```

## Available Scripts

### start_with_foxglove.sh
Main startup script that launches everything needed:
- Starts Docker containers
- Installs dependencies
- Builds workspace
- Launches Gazebo simulation
- Starts Foxglove bridge
- Starts environment visualizer

### demo_pick_place_sequence.sh
Runs automated pick-and-place demonstration by sending joint trajectory commands through 7 motion phases.

### diagnose_simulation.sh
Diagnostic tool that checks:
- Container status
- Running ROS nodes
- Available topics
- Foxglove bridge status
- Joint state publication

## Key ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | sensor_msgs/JointState | Current robot joint positions |
| `/environment_markers` | visualization_msgs/MarkerArray | Block and bin visualization |
| `/robot_description` | std_msgs/String | Robot URDF model |
| `/tf` | tf2_msgs/TFMessage | Transform tree |
| `/set_joint_trajectory` | trajectory_msgs/JointTrajectory | Joint motion commands |
| `/camera/image_raw` | sensor_msgs/Image | Camera feed |

## Troubleshooting

### Robot Not Moving
1. Check if Gazebo is running: `docker ps | grep gazebo`
2. Verify joint states are being published: `ros2 topic hz /joint_states`
3. See FOXGLOVE_COMPLETE_GUIDE.md for detailed troubleshooting

### Can't See Blocks/Bins in Foxglove
1. In Foxglove 3D panel settings, enable `/environment_markers` topic
2. Verify Fixed Frame is set to `world`
3. Check visualizer is running: `ros2 node list | grep environment_visualizer`

### Foxglove Won't Connect
1. Verify Foxglove bridge is running: `ros2 node list | grep foxglove`
2. Check port 8765 is exposed: `docker ps | grep 8765`
3. Try reconnecting to `ws://localhost:8765`

For more detailed troubleshooting, see **FOXGLOVE_COMPLETE_GUIDE.md**

## System Requirements

- **OS**: macOS, Linux, or Windows with WSL2
- **Docker**: Docker Desktop 4.0+
- **Memory**: 4GB+ RAM available for containers
- **Disk**: 10GB+ free space for images and builds

## Development

### Building Changes

After modifying source files:
```bash
docker-compose exec gazebo bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    colcon build &&
    source install/setup.bash
"
```

### Accessing Containers

```bash
# Gazebo container
docker-compose exec gazebo bash

# MoveIt container
docker-compose exec moveit bash

# Source ROS2 environment
source /opt/ros/humble/setup.bash
cd /workspace
source install/setup.bash
```

### Adding New Nodes

1. Add Python file to `workspace/src/ur5_gazebo/ur5_gazebo/`
2. Update `setup.py` with new entry point
3. Rebuild with `colcon build`
4. Run with `ros2 run ur5_gazebo <node_name>`

## Architecture

### Simulation Stack
- **Gazebo** - Physics simulation
- **ROS2 Humble** - Middleware and communication
- **Foxglove Bridge** - WebSocket bridge for visualization
- **Docker** - Containerized deployment

### Robot Control
- **Joint State Publisher** - Publishes current positions
- **Joint Trajectory Controller** - Accepts motion commands
- **Robot State Publisher** - Publishes TF tree
- **Gazebo Plugins** - Joint control interface

### Visualization
- **Foxglove Studio** - Web-based 3D visualization
- **Environment Visualizer** - Publishes object markers
- **Camera Plugin** - Provides image feed

## Future Enhancements

- Full MoveIt integration for collision-aware planning
- Behavioral cloning for learning from demonstrations
- Gripper simulation and grasping physics
- Multiple object manipulation
- Real robot deployment

## Citation

If you use this work in your research:

```bibtex
@misc{armstrong2025,
  title={Armstrong: 6-DOF Pick and Place Robotic Arm Simulation},
  author={Ryan Rahman},
  year={2025},
  howpublished={\url{https://github.com/ryanrahman27/Armstrong-Learning-to-Grasp}}
}
```

## License

Apache 2.0 License
