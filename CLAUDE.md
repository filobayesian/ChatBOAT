# ChatBOAT

Underwater Vehicle-Manipulator System (UVMS) simulation: BlueROV2 Heavy + Reach Alpha 5 arm, using Stonefish simulator + ROS2 Humble. LLM-driven MPC control pipeline.

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

## Build & Run

### ROS2 workspace (on cloud GPU with OpenGL 4.3+)
```bash
colcon build --symlink-install
source install/setup.bash
vglrun -d egl ros2 launch chatboat_bringup bringup.launch.py
```

### MPC bridge (goal-based navigation)
```bash
ros2 launch chatboat_bringup bringup.launch.py use_mpc:=true

# Send a goal:
ros2 topic pub --once /chatboat/mpc_bridge/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: "world"}, pose: {position: {x: 2.0, y: 0.0, z: 2.0}, orientation: {w: 1.0}}}'
```

### llm2control (standalone — requires Python 3.10 with ROS2)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3.10 -m pip install openai numpy casadi
cd src && OPENROUTER_API_KEY=... python3.10 -m llm2control.main
```
**WARNING:** Must use `python3.10` explicitly — ROS2 Humble's rclpy is built for Python 3.10. The default `python` (3.11) cannot load rclpy's C extensions. Running via `poetry run` also fails (venv without rclpy). If you see "rclpy not available — running in MOCK mode", you're using the wrong Python.

## Critical Rules

### Stonefish XML
- All subscriber attributes MUST be on a SINGLE `<ros_subscriber>` element (parser only reads the first one)
- Joint names are namespaced: `chatboat/axis_e`, NOT `axis_e`

### Servo Tuning
- Arm servo gains MUST be low: position_gain=2.0, velocity_gain=3.0, max_torque=3.0
- High gains (>10) cause physics explosion → simulator crash (SIGSEGV)

### Arm Joints
- axis_d/axis_c: NEGATIVE values extend arm forward; positive folds INTO vehicle body

### Thrust Scaling
- Stonefish thrusters expect setpoints in [-1000, 1000], NOT [-1, 1]
- `thruster_mixing()` returns [-1, 1]; must be scaled by `THRUST_SCALE` (default 50.0) before publishing
- `vehicle_teleop` works at setpoints of 10–50; MPC bridge/ros_bridge scale automatically
- Configurable: `thrust_scale` ROS2 parameter on mpc_bridge, or `THRUST_SCALE` in `llm2control/config.py`

### Velocity Frames
- ROS2 Odometry twist is in **body frame** (child_frame_id)
- MPC model uses **world-frame** velocities — must rotate body→world using yaw before storing state
- Both `mpc_bridge.py` and `ros_bridge.py` do this conversion in their odom callbacks

## Key Files
- Stonefish scenario: `src/chatboat_stonefish/scenarios/chatboat_scenario.scn.xml`
- URDF: `src/chatboat_description/urdf/chatboat_uvms.urdf.xacro`
- MPC bridge (ROS2 PD controller): `src/chatboat_control/chatboat_control/mpc_bridge.py`
- Vehicle teleop: `src/chatboat_control/chatboat_control/vehicle_teleop.py`
- Arm teleop: `src/chatboat_control/chatboat_control/arm_teleop.py`
- LLM agent: `src/llm2control/agent.py` (Claude via OpenRouter)
- MPC solver: `src/llm2control/mpc.py` (CasADi/IPOPT, 8D double integrator with CBF)
- ROS bridge: `src/llm2control/ros_bridge.py` (odom subscriber + thruster publisher)
- Config: `src/llm2control/config.py` (topics, workspace bounds, MPC defaults, thrust scale)

## ROS2 Topics
- `/chatboat/thruster_commands` — Float64MultiArray (8 thrusters, scaled setpoints)
- `/chatboat/joint_commands` — JointState (4 joints, namespaced names)
- `/chatboat/joint_states` — JointState (encoder feedback)
- `/chatboat/odometry` — Odometry
- `/chatboat/mpc_bridge/goal` — PoseStamped (navigation target for MPC bridge)
- `/chatboat/mpc_bridge/goal_reached` — Bool (completion status)

## Dependencies
- ROS2 packages: Stonefish simulator (v1.6), stonefish_ros2
- llm2control: openai, numpy, casadi (see src/pyproject.toml)
