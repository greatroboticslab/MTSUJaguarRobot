#!/bin/bash

# Robot Control System - Automated Setup Script
# This script sets up the complete environment and starts the application

set -e  # Exit on any error

echo "🚀 Robot Control System - Automated Setup"
echo "========================================="

# Check if Python 3 is available
if ! command -v python3 &> /dev/null; then
    echo "❌ Error: python3 is not installed or not in PATH"
    echo "   Please install Python 3.10+ from https://python.org"
    exit 1
fi

# Check Python version
PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "✅ Found Python $PYTHON_VERSION"

# Check if version is adequate (3.10+)
if python3 -c "import sys; exit(0 if (sys.version_info.major >= 3 and sys.version_info.minor >= 10) else 1)"; then
    echo "✅ Python version is compatible"
else
    echo "❌ Error: Python 3.10+ required, found $PYTHON_VERSION"
    exit 1
fi

# Create virtual environment if it doesn't exist
if [ ! -d ".venv" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv .venv
    echo "✅ Virtual environment created"
else
    echo "✅ Virtual environment already exists"
fi

# Activate virtual environment
echo "🔄 Activating virtual environment..."
source .venv/bin/activate

# Install dependencies
echo "📥 Installing dependencies..."
pip install -r requirements.txt

# Run setup test
echo "🧪 Running setup verification..."
python3 test_setup.py

echo ""
echo "🎉 Setup complete!"
echo ""
echo "To start the application:"
echo "  1. Run: python3 app.py"
echo "  2. Open browser to: http://127.0.0.1:5014"
echo ""
echo "Or run this script with 'start' to automatically start the app:"
echo "  ./setup.sh start"
echo ""

# Auto-start if requested
if [ "$1" = "start" ]; then
    echo "🚀 Starting application..."
    python3 app.py
fi
