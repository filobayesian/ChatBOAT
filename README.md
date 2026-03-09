# ClawdBoat - UVMS Simulation with LLM-driven MPC Control

Underwater Vehicle-Manipulator System (UVMS) simulation: **BlueROV2 Heavy** + **Reach Alpha 5** arm, using [Stonefish](https://stonefish.readthedocs.io/) simulator and ROS2 Humble. Features an LLM-driven MPC control pipeline that translates natural language commands into optimal vehicle trajectories.

## Project Structure

```
src/
  chatboat_description/   # URDF/xacro robot description (ROS2 ament_cmake)
  chatboat_stonefish/     # Stonefish scenario, meshes, sim config (ROS2 ament_cmake)
  chatboat_control/       # Teleop, MPC bridge, test commander, gripper (ROS2 ament_python)
  chatboat_bringup/       # Top-level launch files (ROS2 ament_cmake)
  llm2control/            # LLM→MPC pipeline (standalone Python, not a ROS2 package)
  dashboard/              # Web dashboard for LLM→MPC control (index.html)
docker/                   # Dockerfile for cloud GPU deployment
scripts/                  # Build/push scripts, mesh converters
```

## Robot Specs

- **Vehicle**: BlueROV2 Heavy, ~11 kg, 8 thrusters (6-DOF: surge/sway/heave/roll/pitch/yaw)
- **Arm**: Reach Alpha 5, 4 revolute DOF, ~0.40 m reach
- **Gripper**: Magnetic sphere (PermanentMagnet)
- **Namespace**: `chatboat`

## RunPod Setup (from scratch)

Stonefish requires **OpenGL 4.3+**. macOS only supports 4.1, so a cloud GPU is needed. We use [RunPod](https://www.runpod.io) (~$0.32/hr for RTX A4000).

### 1. Create Pod

1. Go to [runpod.io/console/pods](https://www.runpod.io/console/pods)
2. Click **+ GPU Pod** → select **RTX A4000** (or any GPU with OpenGL 4.3+)
3. Choose template: **Runpod Pytorch 2.1** (`runpod/pytorch:2.1.0-py3.10-cuda11.8.0-devel-ubuntu22.04`)
   - This gives you Ubuntu 22.04 + Python 3.10 + CUDA — all required for ROS2 Humble
4. **Container Disk**: 40 GB (ROS2 desktop-full is large)
5. Deploy and SSH in

### 2. Install ROS2 Humble

Run each command separately (long lines break when pasting into RunPod terminals):

```bash
apt-get update && apt-get install -y locales software-properties-common curl
```
```bash
locale-gen en_US en_US.UTF-8
```
```bash
export LANG=en_US.UTF-8
```
```bash
add-apt-repository universe
```
```bash
curl -sSL --output /usr/share/keyrings/ros-archive-keyring.gpg https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
```
```bash
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list
```
```bash
apt-get update && apt-get install -y ros-humble-desktop-full python3-colcon-common-extensions
```

### 3. Install GCC 13 (required for latest Stonefish — uses C++20 `<format>`)

```bash
add-apt-repository -y ppa:ubuntu-toolchain-r/test
```
```bash
apt-get update && apt-get install -y gcc-13 g++-13
```

### 4. Install Stonefish dependencies

```bash
apt-get install -y libglm-dev libsdl2-dev libfreetype6-dev libglew-dev libglu1-mesa-dev freeglut3-dev mesa-utils ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher xvfb x11vnc novnc websockify
```

### 5. Install VirtualGL

```bash
wget -q https://github.com/VirtualGL/virtualgl/releases/download/3.1.1/virtualgl_3.1.1_amd64.deb
```
```bash
dpkg -i virtualgl_3.1.1_amd64.deb || apt-get install -f -y
```
```bash
/opt/VirtualGL/bin/vglserver_config -config +s +f -t
```

### 6. Build Stonefish from source (latest main, requires GCC 13)

```bash
cd /opt && git clone --depth 1 https://github.com/patrykcieslak/stonefish.git
```
```bash
cd /opt/stonefish && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-13 -DCMAKE_CXX_COMPILER=g++-13 .. && make -j$(nproc) && make install && ldconfig
```

**Important:** After install, remove the stale config from the source tree (it shadows the real one):

```bash
rm -f /opt/stonefish/StonefishConfig.cmake
```

### 7. Build stonefish_ros2 (use `master` branch — matches latest Stonefish API)

```bash
mkdir -p ~/ChatBOAT/src && cd ~/ChatBOAT/src
```
```bash
git clone --depth 1 --branch master https://github.com/patrykcieslak/stonefish_ros2.git
```

**Patch required** — ROS2 Humble type incompatibility on line 697 of `ROS2Interface.cpp`:

```bash
sed -i 's/msg.header.stamp + rclcpp::Duration/rclcpp::Time(msg.header.stamp) + rclcpp::Duration/' ~/ChatBOAT/src/stonefish_ros2/src/stonefish_ros2/ROS2Interface.cpp
```

Build:

```bash
cd ~/ChatBOAT && source /opt/ros/humble/setup.bash && colcon build --packages-select stonefish_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-13 -DCMAKE_CXX_COMPILER=g++-13
```

> **Version compatibility notes:**
> - Stonefish `main` (latest) requires GCC 13+ for C++20 `<format>` header
> - stonefish_ros2 `master` branch matches latest Stonefish API (`Solver::SI`, `CollisionFilter::EXCLUSIVE`)
> - stonefish_ros2 `dev` branch does NOT match — uses different enum names
> - stonefish_ros2 `v1.3` tag is too old for latest Stonefish
> - The `operator+` patch is needed because ROS2 Humble's `builtin_interfaces::msg::Time` doesn't support `+ rclcpp::Duration` directly

### 8. Clone and build ChatBOAT

```bash
cd ~/ChatBOAT/src && git clone https://github.com/filippogombac/ChatBOAT.git chatboat_src
```
```bash
cp -r ~/ChatBOAT/src/chatboat_src/src/* ~/ChatBOAT/src/
```
```bash
cd ~/ChatBOAT && source /opt/ros/humble/setup.bash && source install/setup.bash
```
```bash
colcon build --symlink-install --packages-select chatboat_description chatboat_stonefish chatboat_control chatboat_bringup
```

### 9. Install LLM pipeline dependencies

```bash
python3.10 -m pip install openai numpy casadi websockets
```

## Running the Simulation

### Terminal 1 — Start virtual display + simulation

```bash
Xvfb :99 -screen 0 1920x1080x24 &
```
```bash
export DISPLAY=:99
```
```bash
export VGL_DISPLAY=egl
```
```bash
source /opt/ros/humble/setup.bash
```
```bash
source ~/ChatBOAT/install/setup.bash
```
```bash
vglrun -d egl ros2 launch chatboat_bringup bringup.launch.py
```

### Terminal 2 — Start LLM→MPC dashboard

```bash
export OPENROUTER_API_KEY=your-key-here
```
```bash
source /opt/ros/humble/setup.bash
```
```bash
source ~/ChatBOAT/install/setup.bash
```
```bash
cd ~/ChatBOAT/src
```
```bash
python3.10 -m llm2control.main --dashboard --port 8765
```

### Terminal 3 — View in browser (noVNC for Stonefish GUI)

On the pod:

```bash
x11vnc -display :99 -forever -nopw -bg
```
```bash
websockify --web=/usr/share/novnc 6080 localhost:5900 &
```

On your **local machine**, open a new terminal and create SSH tunnels:

```bash
ssh -L 6080:localhost:6080 -L 8765:localhost:8765 root@<POD_IP> -p <POD_PORT> -i ~/.ssh/id_ed25519
```

Then open in your browser:
- **Stonefish GUI**: http://localhost:6080/vnc.html
- **LLM Dashboard**: http://localhost:8765

## Alternative: MPC bridge (goal-based navigation without LLM)

```bash
ros2 launch chatboat_bringup bringup.launch.py use_mpc:=true

# Send a goal:
ros2 topic pub --once /chatboat/mpc_bridge/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: "world"}, pose: {position: {x: 2.0, y: 0.0, z: 2.0}, orientation: {w: 1.0}}}'
```

## LLM→MPC Pipeline

A dual-role Claude agent (via OpenRouter) translates natural language into MPC parameters:

1. **Task Planner** — breaks a command into navigation subtasks
2. **Optimization Formulator** — configures MPC weights, constraints, and targets for each subtask
3. **MPC Solver** — CasADi/IPOPT solves a 10D damped double integrator with CBF obstacles
4. **ROS Bridge** — publishes thruster commands at the control rate

### MPC Model

- **State** (10D): `[x, y, z, φ, ψ, ẋ, ẏ, ż, φ̇, ψ̇]` — position, roll, yaw, and their velocities
- **Control** (5D): `[u_x, u_y, u_z, u_φ, u_ψ]` — world-frame accelerations
- **Dynamics**: damped double integrator with linear drag
- **Horizon**: N=15 steps, dt=0.1 s
- Roll stabilisation via cost function weight (`Q_roll`)

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/chatboat/odometry` | `nav_msgs/Odometry` | Vehicle state (twist in body frame) |
| `/chatboat/thruster_commands` | `std_msgs/Float64MultiArray` | 8 thrusters, scaled setpoints |
| `/chatboat/joint_commands` | `sensor_msgs/JointState` | 4 arm joint positions |
| `/chatboat/joint_states` | `sensor_msgs/JointState` | Arm encoder feedback |
| `/chatboat/mpc_bridge/goal` | `geometry_msgs/PoseStamped` | Navigation target |
| `/chatboat/mpc_bridge/goal_reached` | `std_msgs/Bool` | Goal completion status |

## Troubleshooting

### `<format>: No such file or directory` when building Stonefish
Latest Stonefish requires C++20. Install GCC 13: `add-apt-repository -y ppa:ubuntu-toolchain-r/test && apt-get install -y gcc-13 g++-13`, then pass `-DCMAKE_C_COMPILER=gcc-13 -DCMAKE_CXX_COMPILER=g++-13` to cmake.

### `StonefishTargets.cmake` not found when building stonefish_ros2
A stale `StonefishConfig.cmake` in `/opt/stonefish/` shadows the real install at `/usr/local/lib/cmake/Stonefish/`. Fix: `rm -f /opt/stonefish/StonefishConfig.cmake`

### `Solver` / `CollisionFilter` not declared in stonefish_ros2
You're using the wrong branch. Use `master` (not `dev` or `v1.3`).

### `operator+` error on `msg.header.stamp + rclcpp::Duration`
ROS2 Humble incompatibility. Patch: `sed -i 's/msg.header.stamp + rclcpp::Duration/rclcpp::Time(msg.header.stamp) + rclcpp::Duration/' src/stonefish_ros2/src/stonefish_ros2/ROS2Interface.cpp`

### `rclpy not available — running in MOCK mode`
Wrong Python version. Must use `python3.10` explicitly — RunPod's default `python` is 3.11, which can't load ROS2 Humble's rclpy C extensions.

### Simulator crashes with SIGSEGV
Check arm servo gains — values above 10 cause physics explosion on lightweight arm links. Use: `position_gain=2.0, velocity_gain=3.0, max_torque=3.0`.

### RunPod terminal splits long commands
RunPod's web terminal and SSH often break lines at whitespace. Always paste commands as short single lines. Export environment variables separately instead of inline.

## Dependencies

- **ROS2 Humble** (Ubuntu 22.04)
- **Stonefish** (latest main) + stonefish_ros2 (`master` branch)
- **GCC 13** (for C++20 support)
- **VirtualGL 3.1.1**
- **llm2control**: openai, numpy, casadi, websockets
