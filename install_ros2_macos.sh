#!/bin/bash

echo "🤖 Installing ROS2 Humble on macOS (Official Method)"
echo "=================================================="

# Check if we're on macOS
if [[ "$OSTYPE" != "darwin"* ]]; then
    echo "❌ This script is for macOS only"
    exit 1
fi

# Check architecture
ARCH=$(uname -m)
echo "🔍 Detected architecture: $ARCH"

if [[ "$ARCH" == "arm64" ]]; then
    echo "📱 Apple Silicon detected - using ARM64 build"
    ROS2_ARCHIVE="ros2-humble-20241126-macos-amd64.tar.bz2"  # Note: ARM64 builds may not be available
    echo "⚠️  Note: ROS2 ARM64 builds for macOS may be limited"
else
    echo "💻 Intel Mac detected - using AMD64 build"
    ROS2_ARCHIVE="ros2-humble-20241126-macos-amd64.tar.bz2"
fi

# Create ROS2 installation directory
ROS2_INSTALL_DIR="/opt/ros/humble"
echo "📁 Installing to: $ROS2_INSTALL_DIR"

# Check if already installed
if [ -d "$ROS2_INSTALL_DIR" ]; then
    echo "✅ ROS2 appears to be already installed at $ROS2_INSTALL_DIR"
    echo "🔧 To reinstall, remove the directory first: sudo rm -rf $ROS2_INSTALL_DIR"
else
    echo "❌ ROS2 not found. You'll need to install it manually."
    echo ""
    echo "📋 Manual Installation Steps:"
    echo "1. Go to: https://github.com/ros2/ros2/releases"
    echo "2. Download the latest macOS release (ros2-humble-*-macos-amd64.tar.bz2)"
    echo "3. Extract: tar xf ~/Downloads/ros2-humble-*-macos-amd64.tar.bz2"
    echo "4. Move to system: sudo mv ros2-osx /opt/ros/humble"
    echo "5. Install dependencies: brew install python@3.11 cmake"
    echo ""
    echo "🔗 Alternative: Use conda-forge"
    echo "conda install -c conda-forge ros-humble-desktop"
fi

# Check for Python and other dependencies
echo ""
echo "🔍 Checking dependencies..."

if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version | cut -d' ' -f2 | cut -d'.' -f1,2)
    echo "✅ Python3: $PYTHON_VERSION"
else
    echo "❌ Python3 not found - install with: brew install python@3.11"
fi

if command -v cmake &> /dev/null; then
    echo "✅ CMake: $(cmake --version | head -1)"
else
    echo "❌ CMake not found - install with: brew install cmake"
fi

# Install dependencies if missing
echo ""
echo "📦 Installing required dependencies..."
brew install python@3.11 cmake qt@5 || echo "⚠️  Some dependencies may already be installed"

echo ""
echo "🎯 Next Steps:"
echo "1. Download ROS2: https://github.com/ros2/ros2/releases"
echo "2. Or try conda: conda install -c conda-forge ros-humble-desktop"
echo "3. Then run: source /opt/ros/humble/setup.bash"
echo "4. Test with: ros2 --help"