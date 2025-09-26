#!/bin/bash

echo "🧪 Simple Armstrong Test (No Launch Files)"
echo "=========================================="

# Start containers
echo "🚀 Starting containers..."
docker-compose up -d gazebo moveit

# Wait for startup
echo "⏳ Waiting for containers to start..."
sleep 15

# Test if we can access ROS2 inside the container
echo "🔍 Testing ROS2 connectivity..."
if docker-compose exec gazebo ros2 node list; then
    echo "✅ ROS2 is working inside containers"
else
    echo "❌ ROS2 connection failed"
fi

echo "📋 Container status:"
docker-compose ps

echo "🧹 Stopping containers..."
docker-compose down

echo "✅ Simple test completed!"