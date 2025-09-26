#!/bin/bash

echo "🖥️  Setting up macOS GUI support for RViz2"
echo "========================================"

# Check if XQuartz is installed
if ! command -v xquartz &> /dev/null; then
    echo "📦 Installing XQuartz..."
    if command -v brew &> /dev/null; then
        brew install --cask xquartz
    else
        echo "❌ Homebrew not found. Please install XQuartz manually:"
        echo "   Visit: https://www.xquartz.org/"
        echo "   Or install Homebrew first: /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
        exit 1
    fi
else
    echo "✅ XQuartz is already installed"
fi

echo ""
echo "🔧 Configuration steps:"
echo "1. Start XQuartz: open -a XQuartz"
echo "2. In XQuartz preferences:"
echo "   - Go to Security tab"
echo "   - Check 'Allow connections from network clients'"
echo "3. Restart XQuartz"
echo "4. Set display variable: export DISPLAY=:0"
echo ""
echo "🚀 Then run: docker-compose up rviz"