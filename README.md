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
docker/                   # Dockerfile for cloud GPU deployment
scripts/                  # Build/push scripts, mesh converters
```

## Robot Specs

- **Vehicle**: BlueROV2 Heavy, ~11 kg, 8 thrusters (6-DOF: surge/sway/heave/roll/pitch/yaw)
- **Arm**: Reach Alpha 5, 4 revolute DOF, ~0.40 m reach
- **Gripper**: Magnetic sphere (PermanentMagnet)
- **Namespace**: `chatboat`

## Quick Start

### Prerequisites

Stonefish requires **OpenGL 4.3+**. macOS only supports 4.1, so a cloud GPU is needed.

We use [RunPod](https://www.runpod.io) with a GPU pod (~$0.32/hr for RTX A4000).

### 1. Build and push the Docker image

```bash
docker login
./scripts/build_and_push.sh <your-dockerhub-username>
```

### 2. Create a RunPod Pod

1. Go to [runpod.io/console/pods](https://www.runpod.io/console/pods)
2. Click **+ GPU Pod** → select **RTX A4000**
3. Change template to **Custom**:
   - **Container Image**: `<your-dockerhub-username>/chatboat-sim:latest`
   - **Expose HTTP Ports**: `6080`
   - **Container Disk**: 20 GB
4. Click **Deploy On-Demand**

### 3. Open the simulation

Once the pod is running, click **Connect** → **HTTP [6080]** to open the Stonefish GUI via noVNC.

### 4. Launch the simulation

SSH into the pod, then:

```bash
colcon build --symlink-install
source install/setup.bash
export DISPLAY=:99 && export VGL_DISPLAY=egl
vglrun -d egl ros2 launch chatboat_bringup bringup.launch.py
```

### 5. MPC bridge (goal-based navigation)

```bash
ros2 launch chatboat_bringup bringup.launch.py use_mpc:=true

# Send a goal:
ros2 topic pub --once /chatboat/mpc_bridge/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: "world"}, pose: {position: {x: 2.0, y: 0.0, z: 2.0}, orientation: {w: 1.0}}}'
```

### 6. LLM→MPC pipeline

The LLM pipeline is a standalone Python package — **must use `python3.10`** (ROS2 Humble's rclpy is built for 3.10).

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3.10 -m pip install openai numpy casadi
cd src && OPENROUTER_API_KEY=... python3.10 -m llm2control.main
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

## Development

For faster iteration, SSH into the pod and edit + rebuild directly:

```bash
cd ~/ChatBOAT && colcon build --symlink-install && source install/setup.bash
```

## Dependencies

- **ROS2 Humble** (Ubuntu 22.04)
- **Stonefish v1.6** + stonefish_ros2
- **llm2control**: openai, numpy, casadi
