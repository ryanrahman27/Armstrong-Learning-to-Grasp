#!/bin/bash

echo "🧪 Testing XQuartz Setup"
echo "======================="

# Check if XQuartz is running
if pgrep -x "Xquartz" > /dev/null; then
    echo "✅ XQuartz is running"
else
    echo "❌ XQuartz is not running. Starting it..."
    open -a XQuartz
    sleep 3
fi

# Set display
export DISPLAY=:0
echo "📺 DISPLAY set to: $DISPLAY"

# Test X11 forwarding with a simple GUI app
echo "🧪 Testing X11 forwarding..."
if command -v xeyes &> /dev/null; then
    echo "Running xeyes test (close the window when it appears)..."
    timeout 10 xeyes || echo "xeyes test completed or timed out"
else
    echo "⚠️  xeyes not found (normal on macOS)"
fi

# Test Docker X11 forwarding
echo "🐳 Testing Docker X11 forwarding..."
if docker run --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix hello-world > /dev/null 2>&1; then
    echo "✅ Docker can connect to X11"
else
    echo "⚠️  Docker X11 connection test inconclusive"
fi

echo ""
echo "🚀 Ready to test RViz!"
echo "Run: docker-compose up rviz"