#!/bin/bash

# Armstrong Quick Start Script
# Simple version for testing the setup

set -e

echo "🚀 Armstrong Quick Start"
echo "======================="

# Check Docker
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker Desktop first."
    exit 1
fi

# Step 1: Start containers
echo "Step 1: Starting containers..."
docker-compose up -d

echo "Step 2: Waiting for services to be ready..."
sleep 15

# Step 3: Check if services are running
echo "Step 3: Checking services..."
echo "Gazebo container:"
docker-compose ps gazebo

echo "MoveIt container:"
docker-compose ps moveit

# Step 4: Show logs
echo "Step 4: Showing container logs (last 10 lines each)..."
echo "=== Gazebo Logs ==="
docker-compose logs --tail=10 gazebo

echo "=== MoveIt Logs ==="
docker-compose logs --tail=10 moveit

echo ""
echo "✅ Setup complete!"
echo ""
echo "🔧 Available commands:"
echo "  View live logs:     docker-compose logs -f gazebo"
echo "  Enter container:    docker-compose exec gazebo bash"
echo "  Stop containers:    docker-compose down"
echo "  Start RViz:         docker-compose up rviz"
echo ""
echo "📊 To run full experiment:"
echo "  ./run_experiment.sh"