# Armstrong: 6-DOF Pick and Place Comparison

This project implements and compares two approaches for robotic pick-and-place tasks:
1. **IK + MoveIt Planning**: Traditional motion planning with inverse kinematics
2. **Imitation Learning**: Behavioral cloning from human demonstrations

## Project Structure

```
Armstrong/
├── docker-compose.yml           # Multi-container setup
├── workspace/
│   └── src/
│       ├── ur5_gazebo/         # Gazebo simulation
│       ├── ur5_moveit_config/  # MoveIt2 configuration
│       └── bc_training/        # Behavioral cloning training
├── run_experiment.sh           # Complete experiment runner
└── README.md                   # This file
```

## Features

### Simulation Environment
- UR5 6-DOF robotic arm in Gazebo
- Colored block detection with computer vision
- Pick-and-place world with bin target
- Real-time physics simulation

### MoveIt Integration
- Complete MoveIt2 configuration
- OMPL motion planning
- Collision-aware trajectory generation
- RViz visualization

### Imitation Learning
- ROS bag recording of human demonstrations
- Behavioral cloning with TensorFlow
- Neural network policy training
- Real-time policy execution

### Comparison Framework
- Automated evaluation system
- Success rate metrics
- Completion time analysis
- Statistical comparison

## Quick Start

### 1. Launch Complete System
```bash
# Start all containers
docker-compose up -d

# Run complete experiment
./run_experiment.sh
```

### 2. Individual Components

#### Gazebo Simulation
```bash
docker-compose up gazebo
```

#### MoveIt Planning
```bash
docker-compose up moveit
```

#### RViz Visualization (macOS native)
```bash
# Install ROS2 Humble on macOS first
ros2 launch ur5_moveit_config rviz.launch.py
```

## Recording Demonstrations

### 1. Start Recording System
```bash
ros2 run ur5_gazebo demo_recorder.py
```

### 2. Control Robot Manually
```bash
# Start recording
ros2 topic pub /record_demo std_msgs/msg/Bool "data: true"

# Use manual control or joystick to demonstrate pick-and-place
# ... perform demonstrations ...

# Stop recording
ros2 topic pub /record_demo std_msgs/msg/Bool "data: false"
```

### 3. Train BC Policy
```bash
# Train on recorded demonstrations
docker-compose run bc_training python3 train_policy.py \
    --bag_path /data/demonstrations/demo_YYYYMMDD_HHMMSS \
    --model_path /data/bc_model.h5 \
    --epochs 100
```

## Running Comparisons

### 1. Start Evaluation System
```bash
# Launch all required nodes
ros2 run ur5_gazebo pick_place_controller.py &
ros2 run ur5_gazebo bc_policy_controller.py &
ros2 run ur5_gazebo comparison_evaluator.py &
```

### 2. Evaluate MoveIt Method
```bash
ros2 topic pub /start_evaluation std_msgs/msg/String "data: 'moveit'"
```

### 3. Evaluate BC Method
```bash
ros2 topic pub /start_evaluation std_msgs/msg/String "data: 'bc'"
```

### 4. View Results
Results are automatically saved to `/data/evaluation_*.json`

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | sensor_msgs/JointState | Current robot joint positions |
| `/block_pose` | geometry_msgs/PoseStamped | Detected block position |
| `/record_demo` | std_msgs/Bool | Start/stop demonstration recording |
| `/bc_policy_active` | std_msgs/Bool | Activate/deactivate BC policy |
| `/start_evaluation` | std_msgs/String | Start evaluation ("moveit" or "bc") |
| `/evaluation_status` | std_msgs/String | Current evaluation status |

## Configuration

### MoveIt Parameters
- **Planner**: RRTConnect (default)
- **Planning Time**: 5.0 seconds
- **Velocity Scaling**: 0.1
- **Acceleration Scaling**: 0.1

### BC Training Parameters
- **Architecture**: 256-256-128 fully connected
- **Activation**: ReLU with dropout (0.2)
- **Optimizer**: Adam
- **Loss**: Mean Squared Error
- **Early Stopping**: 10 epochs patience

### Evaluation Parameters
- **Trials per Method**: 10
- **Success Threshold**: 5cm from bin center
- **Timeout**: 60 seconds per trial

## Results Interpretation

The system generates detailed comparison metrics:

- **Success Rate**: Percentage of successful pick-and-place operations
- **Completion Time**: Average time for successful trials
- **Robustness**: Consistency across multiple trials
- **Failure Analysis**: Categorization of failure modes

## Development

### Adding New Evaluation Metrics
1. Modify `comparison_evaluator.py`
2. Add new metric calculation in `record_trial_result()`
3. Update report generation

### Extending BC Architecture
1. Edit `train_policy.py`
2. Modify `build_model()` function
3. Adjust hyperparameters

### Adding New Planners
1. Update `ompl_planning.yaml`
2. Add planner configuration
3. Set as default in MoveIt config

## Troubleshooting

### Common Issues

1. **Container Connection Issues**
   ```bash
   # Restart containers
   docker-compose down && docker-compose up -d
   ```

2. **MoveIt Planning Failures**
   ```bash
   # Check joint limits and collision geometry
   ros2 topic echo /move_group/status
   ```

3. **BC Training Convergence**
   ```bash
   # Increase training data or adjust learning rate
   # Check demonstration quality and synchronization
   ```

### Performance Optimization

- Adjust MoveIt planning parameters for speed vs. success rate
- Tune BC network architecture for your specific task
- Optimize computer vision parameters for your lighting conditions

## Citation

If you use this work in your research, please cite:

```bibtex
@misc{armstrong2024,
  title={Armstrong: Learning to Grasp},
  author={Ryan Rahman},
  year={2025},
  howpublished={\\url{https://github.com/ryanrahman27/Armstrong-Learning-to-Grasp}}
}
```

## License

Apache 2.0 License - see LICENSE file for details.