#!/bin/bash
set -e

echo "=== ChatBOAT Simulation Starting ==="

# ---------- SSH (for RunPod) ----------
if [ -n "$RUNPOD_POD_ID" ] || [ -n "$PUBLIC_KEY" ]; then
    echo "RunPod detected, starting SSH..."
    if [ -n "$PUBLIC_KEY" ]; then
        mkdir -p /root/.ssh
        echo "$PUBLIC_KEY" >> /root/.ssh/authorized_keys
        chmod 700 /root/.ssh && chmod 600 /root/.ssh/authorized_keys
    fi
    /usr/sbin/sshd
fi

# ---------- Virtual display + VNC ----------
Xvfb :99 -screen 0 1920x1080x24 +extension GLX &
sleep 1

# Configure VirtualGL to use the GPU
/opt/VirtualGL/bin/vglserver_config -config +s +f -t 2>/dev/null || true

# Start VNC server
x11vnc -display :99 -forever -nopw -shared -rfbport 5900 &

# Start noVNC (web client)
websockify --web /usr/share/novnc 6080 localhost:5900 &

echo "=== noVNC available at http://localhost:6080 ==="

# ---------- GPU check ----------
echo "GPU status:"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null || echo "  (nvidia-smi not available — will use software rendering)"
echo "OpenGL renderer:"
/opt/VirtualGL/bin/vglrun glxinfo 2>/dev/null | grep "OpenGL renderer" || echo "  (glxinfo not available)"

# ---------- ROS2 ----------
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Rebuild if source was volume-mounted (dev workflow)
if [ -f /ros2_ws/src/chatboat_description/package.xml ]; then
    cd /ros2_ws
    colcon build --packages-select \
      chatboat_description \
      chatboat_stonefish \
      chatboat_control \
      chatboat_bringup \
      --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5
    source /ros2_ws/install/setup.bash
fi

echo "=== ROS2 workspace ready ==="

# Launch with VirtualGL so Stonefish gets GPU-accelerated OpenGL
if [ "$1" = "ros2" ]; then
    exec /opt/VirtualGL/bin/vglrun "$@"
else
    exec "$@"
fi
