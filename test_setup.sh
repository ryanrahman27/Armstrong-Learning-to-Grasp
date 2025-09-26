#!/bin/bash

echo "🧪 Testing Armstrong Setup"
echo "=========================="

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker Desktop first."
    exit 1
fi
echo "✅ Docker is running"

# Check if Docker Compose is available
if ! command -v docker-compose &> /dev/null; then
    echo "❌ docker-compose not found. Please install Docker Compose."
    exit 1
fi
echo "✅ docker-compose is available"

# Test building the containers
echo "🔨 Testing container builds..."
if docker-compose build > /dev/null 2>&1; then
    echo "✅ Containers build successfully"
else
    echo "❌ Container build failed"
    echo "Running docker-compose build for detailed output:"
    docker-compose build
    exit 1
fi

# Test that all required images are available
echo "📦 Checking Docker images..."
REQUIRED_IMAGES=("osrf/ros:humble-desktop-full" "tensorflow/tensorflow:2.13.0-gpu")
for image in "${REQUIRED_IMAGES[@]}"; do
    if docker image inspect $image > /dev/null 2>&1; then
        echo "✅ Image available: $image"
    else
        echo "⚠️  Image not available locally (will be downloaded): $image"
    fi
done

echo ""
echo "🎉 Setup test completed successfully!"
echo ""
echo "To run the full experiment:"
echo "1. docker-compose up -d"
echo "2. ./run_experiment.sh"
echo ""
echo "To run individual components:"
echo "- Gazebo only: docker-compose up gazebo"
echo "- MoveIt only: docker-compose up moveit"
echo "- RViz only: docker-compose up rviz"