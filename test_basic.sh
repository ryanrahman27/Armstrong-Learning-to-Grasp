#!/bin/bash

echo "🧪 Basic Armstrong Test"
echo "====================="

# Check Docker
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker Desktop first."
    exit 1
fi
echo "✅ Docker is running"

# Test Gazebo container build
echo "🔨 Testing Gazebo container build..."
if docker-compose build gazebo > /dev/null 2>&1; then
    echo "✅ Gazebo container builds successfully"
else
    echo "❌ Gazebo container build failed"
    echo "Running detailed build:"
    docker-compose build gazebo
    exit 1
fi

# Test starting Gazebo
echo "🚀 Testing Gazebo startup..."
docker-compose up -d gazebo

# Wait a bit
sleep 10

# Check if container is running
if docker-compose ps gazebo | grep -q "Up"; then
    echo "✅ Gazebo container is running"
else
    echo "❌ Gazebo container failed to start"
    echo "Container logs:"
    docker-compose logs gazebo
    exit 1
fi

# Test package build inside container
echo "📦 Testing package build..."
if docker-compose exec gazebo bash -c "cd /workspace && colcon build --packages-select ur5_gazebo" > /dev/null 2>&1; then
    echo "✅ ur5_gazebo package builds successfully"
else
    echo "❌ Package build failed, checking logs:"
    docker-compose exec gazebo bash -c "cd /workspace && colcon build --packages-select ur5_gazebo --event-handlers console_direct+"
fi

# Cleanup
echo "🧹 Cleaning up..."
docker-compose down

echo "✅ Basic test completed!"