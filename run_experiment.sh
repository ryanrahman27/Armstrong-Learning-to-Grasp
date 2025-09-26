#!/bin/bash

# Armstrong Pick and Place Experiment Runner
# This script runs the complete comparison between MoveIt and BC policy

set -e

echo "=== Armstrong Pick and Place Experiment ==="
echo "Comparing IK + MoveIt planning vs Imitation Learning"
echo

# Create necessary directories
mkdir -p data/demonstrations
mkdir -p data/models
mkdir -p data/results

# Function to wait for services
wait_for_service() {
    echo "Waiting for $1 to be available..."
    while ! docker-compose exec gazebo ros2 service list | grep -q "$1" 2>/dev/null; do
        sleep 1
    done
    echo "$1 is ready"
}

# Function to run evaluation
run_evaluation() {
    local method=$1
    echo "Running evaluation for $method method..."
    
    # Start evaluation
    docker-compose exec gazebo ros2 topic pub --once /start_evaluation std_msgs/msg/String "data: '$method'" &
    
    # Wait for completion (monitor status topic)
    echo "Monitoring evaluation progress..."
    timeout 600 bash -c "
        while true; do
            status=\$(docker-compose exec gazebo ros2 topic echo --once /evaluation_status 2>/dev/null || echo '')
            if [[ \$status == *'${method}_completed'* ]]; then
                echo 'Evaluation completed for $method'
                break
            fi
            sleep 5
        done
    "
}

# Step 1: Build workspace (done in containers)
echo "Step 1: Building workspace..."
echo "Packages will be built inside containers automatically"

# Step 2: Start Gazebo simulation
echo "Step 2: Starting Gazebo simulation..."
docker-compose up -d gazebo

# Wait for Gazebo to be ready
sleep 10

# Step 3: Start MoveIt
echo "Step 3: Starting MoveIt..."
docker-compose up -d moveit

# Wait for MoveIt to be ready
sleep 10
wait_for_service "/plan_kinematic_path"

# Step 4: Record demonstrations (if requested)
if [[ "$1" == "--record-demos" ]]; then
    echo "Step 4: Recording human demonstrations..."
    echo "Use joystick/manual control to demonstrate pick-and-place"
    echo "Send 'true' to /record_demo to start recording"
    echo "Send 'false' to /record_demo to stop recording"
    echo "Press Enter when done recording demonstrations..."
    read -r
fi

# Step 5: Train BC policy (if demonstrations exist)
if ls data/demonstrations/demo_*.db3 1> /dev/null 2>&1; then
    echo "Step 5: Training BC policy..."
    
    # Find the latest demonstration
    latest_demo=$(ls -t data/demonstrations/demo_*.db3 | head -n1)
    demo_path=$(dirname "$latest_demo")/$(basename "$latest_demo" .db3)
    
    # Train the model
    docker-compose run --rm bc_training python3 /workspace/src/bc_training/train_policy.py \
        --bag_path "$demo_path" \
        --model_path "/data/models/bc_model.h5" \
        --epochs 100
else
    echo "Step 5: No demonstrations found, using pre-trained model..."
    # In a real scenario, you would have a pre-trained model
fi

# Step 6: Start evaluation nodes
echo "Step 6: Starting evaluation system..."

# Start pick-place controller
docker-compose exec gazebo ros2 run ur5_gazebo pick_place_controller &
PICKPLACE_PID=$!

# Start BC policy controller
docker-compose exec gazebo ros2 run ur5_gazebo bc_policy_controller \
    --ros-args -p model_path:=/data/models/bc_model.h5 &
BC_PID=$!

# Start comparison evaluator
docker-compose exec gazebo ros2 run ur5_gazebo comparison_evaluator \
    --ros-args -p num_trials:=10 &
EVAL_PID=$!

sleep 5

# Step 7: Run MoveIt evaluation
echo "Step 7: Evaluating MoveIt method..."
run_evaluation "moveit"

# Step 8: Run BC evaluation
echo "Step 8: Evaluating BC method..."
run_evaluation "bc"

# Step 9: Generate comparison report
echo "Step 9: Generating comparison report..."

# Find latest result files
moveit_result=$(ls -t data/evaluation_moveit_*.json | head -n1)
bc_result=$(ls -t data/evaluation_bc_*.json | head -n1)

# Create summary report
python3 << EOF
import json

# Load results
with open('$moveit_result', 'r') as f:
    moveit_data = json.load(f)

with open('$bc_result', 'r') as f:
    bc_data = json.load(f)

# Generate comparison
print("\\n=== FINAL COMPARISON RESULTS ===")
print(f"MoveIt + IK Planning:")
print(f"  Success Rate: {moveit_data['success_rate']:.1%}")
print(f"  Avg Time: {moveit_data['average_completion_time']:.2f}s")
print(f"\\nBehavioral Cloning:")
print(f"  Success Rate: {bc_data['success_rate']:.1%}")
print(f"  Avg Time: {bc_data['average_completion_time']:.2f}s")
print(f"\\nComparison:")
success_diff = bc_data['success_rate'] - moveit_data['success_rate']
time_diff = bc_data['average_completion_time'] - moveit_data['average_completion_time']
print(f"  BC vs MoveIt Success Rate: {success_diff:+.1%}")
print(f"  BC vs MoveIt Time: {time_diff:+.2f}s")

if success_diff > 0:
    print("\\n🎉 Behavioral Cloning achieved higher success rate!")
else:
    print("\\n🤖 MoveIt achieved higher success rate!")

if time_diff < 0:
    print("⚡ Behavioral Cloning was faster on average!")
else:
    print("⏱️  MoveIt was faster on average!")
EOF

# Cleanup
echo "Cleaning up processes..."
kill $PICKPLACE_PID $BC_PID $EVAL_PID 2>/dev/null || true

echo "\\n=== Experiment Complete ==="
echo "Results saved in data/evaluation_*.json"
echo "View with: docker-compose run --rm rviz"