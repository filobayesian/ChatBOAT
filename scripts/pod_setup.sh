#!/bin/bash
# Direct install on a RunPod pod (Ubuntu 22.04 + NVIDIA GPU).
# Usage: cd ~/ChatBOAT && bash scripts/pod_setup.sh
set -euo pipefail

echo "=== ChatBOAT Pod Setup ==="
echo "This will install ROS2 Humble, Stonefish, VNC, and build the workspace."
echo ""

export DEBIAN_FRONTEND=noninteractive

# ---------- ROS2 Humble ----------
if ! command -v ros2 &> /dev/null; then
    echo ">>> Installing ROS2 Humble..."
    apt-get update
    apt-get install -y software-properties-common curl
    add-apt-repository -y universe
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list
    apt-get update
    apt-get install -y ros-humble-desktop \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        python3-colcon-common-extensions \
        python3-rosdep
    # Init rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        rosdep init || true
    fi
    rosdep update || true
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
else
    echo ">>> ROS2 already installed, skipping."
fi

source /opt/ros/humble/setup.bash

# ---------- Stonefish dependencies ----------
echo ">>> Installing Stonefish dependencies..."
apt-get install -y --no-install-recommends \
    git cmake build-essential \
    libglm-dev libsdl2-dev libfreetype6-dev \
    libglew-dev libglu1-mesa-dev freeglut3-dev \
    mesa-utils

# ---------- VNC stack ----------
echo ">>> Installing VNC/noVNC..."
apt-get install -y --no-install-recommends \
    x11vnc xvfb novnc websockify

# ---------- VirtualGL ----------
if ! command -v vglrun &> /dev/null; then
    echo ">>> Installing VirtualGL..."
    wget -q https://github.com/VirtualGL/virtualgl/releases/download/3.1.1/virtualgl_3.1.1_amd64.deb
    dpkg -i virtualgl_3.1.1_amd64.deb || apt-get install -f -y
    rm virtualgl_3.1.1_amd64.deb
    /opt/VirtualGL/bin/vglserver_config -config +s +f -t 2>/dev/null || true
else
    echo ">>> VirtualGL already installed, skipping."
fi

# ---------- Build Stonefish library ----------
if [ ! -f /usr/local/lib/libStonefish.so ]; then
    echo ">>> Building Stonefish from source..."
    cd /opt
    git clone --depth 1 https://github.com/patrykcieslak/stonefish.git
    cd stonefish
    mkdir -p build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j$(nproc)
    make install
    ldconfig
else
    echo ">>> Stonefish library already installed, skipping."
fi

# ---------- ROS2 workspace ----------
echo ">>> Setting up ROS2 workspace..."
WSDIR=~/chatboat_ws
mkdir -p $WSDIR/src

# Symlink project packages
for pkg in chatboat_description chatboat_stonefish chatboat_control chatboat_bringup; do
    ln -sfn ~/ChatBOAT/src/$pkg $WSDIR/src/$pkg
done

# Clone stonefish_ros2
if [ ! -d "$WSDIR/src/stonefish_ros2" ]; then
    cd $WSDIR/src
    git clone --depth 1 https://github.com/patrykcieslak/stonefish_ros2.git
fi

# Build everything
echo ">>> Building ROS2 workspace..."
cd $WSDIR
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source $WSDIR/install/setup.bash" >> ~/.bashrc

# ---------- Done ----------
echo ""
echo "==========================================="
echo "  Setup complete!"
echo "==========================================="
echo ""
echo "Next steps:"
echo ""
echo "  1. Source the workspace:"
echo "     source ~/.bashrc"
echo ""
echo "  2. Start the VNC display (run this in background):"
echo "     bash ~/ChatBOAT/scripts/start_vnc.sh &"
echo ""
echo "  3. Open noVNC in your browser:"
echo "     In RunPod: Connect -> HTTP [6080]"
echo ""
echo "  4. Launch the simulation:"
echo "     vglrun ros2 launch chatboat_bringup bringup.launch.py"
echo ""
echo "  5. Run the cube stacking demo:"
echo "     vglrun ros2 launch chatboat_bringup bringup.launch.py run_demo:=true"
echo ""
