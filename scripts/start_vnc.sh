#!/bin/bash
# Start the VNC display server for Stonefish GUI.
# Run this before launching the simulation.
# View at: RunPod Connect -> HTTP [6080]
set -e

export DISPLAY=:99

# Kill any existing instances
pkill -f "Xvfb :99" 2>/dev/null || true
pkill -f x11vnc 2>/dev/null || true
pkill -f websockify 2>/dev/null || true
sleep 1

# Virtual framebuffer
Xvfb :99 -screen 0 1920x1080x24 +extension GLX &
sleep 1

# VNC server
x11vnc -display :99 -forever -nopw -shared -rfbport 5900 &

# noVNC web client on port 6080
websockify --web /usr/share/novnc 6080 localhost:5900 &

echo "=== VNC ready ==="
echo "Display: $DISPLAY"
echo "noVNC:   http://localhost:6080"
echo ""
echo "GPU OpenGL check:"
/opt/VirtualGL/bin/vglrun glxinfo 2>/dev/null | grep -E "OpenGL (renderer|version)" || echo "(glxinfo not available)"
echo ""
echo "Now launch Stonefish with:  vglrun ros2 launch chatboat_bringup bringup.launch.py"
