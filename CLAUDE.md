# ChatBOAT

Underwater Vehicle-Manipulator System (UVMS) simulation: BlueROV2 Heavy + Reach Alpha 5 arm, using Stonefish simulator + ROS2 Humble. LLM-driven MPC control pipeline.

## Project Structure

```
src/
  chatboat_description/   # URDF/xacro robot description (ROS2 ament_cmake)
  chatboat_stonefish/     # Stonefish scenario, meshes, sim config (ROS2 ament_cmake)
  chatboat_control/       # Teleop, test commander, gripper service (ROS2 ament_python)
  chatboat_bringup/       # Top-level launch files (ROS2 ament_cmake)
  llm2control/            # LLM→MPC pipeline (standalone Python/Poetry, not a ROS2 package)
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

### llm2control (standalone)
```bash
cd src && poetry install
OPENROUTER_API_KEY=... python -m llm2control.main
```

## Critical Rules

### Stonefish XML
- All subscriber attributes MUST be on a SINGLE `<ros_subscriber>` element (parser only reads the first one)
- Joint names are namespaced: `chatboat/axis_e`, NOT `axis_e`

### Servo Tuning
- Arm servo gains MUST be low: position_gain=2.0, velocity_gain=3.0, max_torque=3.0
- High gains (>10) cause physics explosion → simulator crash (SIGSEGV)

### Arm Joints
- axis_d/axis_c: NEGATIVE values extend arm forward; positive folds INTO vehicle body

## Key Files
- Stonefish scenario: `src/chatboat_stonefish/scenarios/chatboat_scenario.scn.xml`
- URDF: `src/chatboat_description/urdf/chatboat_uvms.urdf.xacro`
- Arm teleop: `src/chatboat_control/chatboat_control/arm_teleop.py`
- LLM agent: `src/llm2control/agent.py` (Claude via OpenRouter)
- MPC solver: `src/llm2control/mpc.py` (CVXPY/OSQP, 1D double integrator)

## ROS2 Topics
- `/chatboat/thruster_commands` — Float64MultiArray (8 thrusters, [-1,1])
- `/chatboat/joint_commands` — JointState (4 joints, namespaced names)
- `/chatboat/joint_states` — JointState (encoder feedback)
- `/chatboat/odometry` — Odometry

## Dependencies
- ROS2 packages: Stonefish simulator (v1.6), stonefish_ros2
- llm2control: openai, numpy, cvxpy (see src/pyproject.toml)
