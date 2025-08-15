#!/bin/bash

# Simple launcher script for the Robot Control System
# This script starts the application from the root directory

# Change to the directory where this script is located
cd "$(dirname "${BASH_SOURCE[0]}")"

# Activate virtual environment if it exists
if [ -d ".venv" ]; then
    echo "🔄 Activating virtual environment..."
    source .venv/bin/activate
fi

echo "🚀 Starting Robot Control System..."
echo "📍 Application will be available at: http://127.0.0.1:5014"
echo "⏹️  Press Ctrl+C to stop"
echo ""

# Start the application
python3 src/app.py
