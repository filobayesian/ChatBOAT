"""LLM-MPC pipeline orchestrator.

Usage:
  OPENROUTER_API_KEY=... python -m llm2control.main

Modes:
  - With ROS2 (rclpy available): reads odometry, publishes thruster commands
  - Without ROS2: uses mock state propagation for offline testing
"""

import os
import time

import numpy as np

from llm2control.agent import LaMPCAgent
from llm2control.config import ROBOT_START, KNOWN_OBJECTS, MPC_DT
from llm2control.dynamics import world_to_body, vehicle_dynamics_matrices, thruster_mixing
from llm2control.mpc import VehicleMPCSolver
from llm2control.ros_bridge import ROSBridge


def _build_scene_description(state: np.ndarray | None) -> str:
    """Build a human-readable scene description for logging."""
    if state is not None:
        pos = f"({state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f})"
    else:
        pos = f"({ROBOT_START[0]:.1f}, {ROBOT_START[1]:.1f}, {ROBOT_START[2]:.1f})"

    lines = [f"Vehicle at {pos}"]
    for obj in KNOWN_OBJECTS:
        p = obj["position"]
        lines.append(f"  {obj['name']}: ({p[0]}, {p[1]}, {p[2]})")
    return "\n".join(lines)


def main():
    api_key = os.environ.get("OPENROUTER_API_KEY")
    if not api_key:
        print("Error: OPENROUTER_API_KEY environment variable not set")
        return

    # ── Initialise components ────────────────────────────────────────────
    ros = ROSBridge()
    solver = VehicleMPCSolver(dt=MPC_DT, horizon=15)
    agent = LaMPCAgent(api_key=api_key)

    # Dynamics matrices for offline mode
    A_dyn, B_dyn = vehicle_dynamics_matrices(MPC_DT)

    # Wait for first odometry (give simulator time to start)
    print("Waiting for odometry...")
    for _ in range(50):  # 5 seconds
        ros.spin_once(timeout_sec=0.1)
        if ros.get_vehicle_state() is not None:
            break
    else:
        print("No odometry received — running in offline (mock) mode")

    use_ros = ros.get_vehicle_state() is not None
    mock_state = np.zeros(8)
    mock_state[:3] = ROBOT_START
    if not use_ros:
        print(f"Mock start state: {mock_state[:4]}")

    # ── User input ───────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print(_build_scene_description(ros.get_vehicle_state()))
    print("=" * 60)
    user_query = input("\nEnter navigation command: ").strip()
    if not user_query:
        print("No command entered.")
        ros.shutdown()
        return

    # ── Task Planning ────────────────────────────────────────────────────
    state = ros.get_vehicle_state() if use_ros else mock_state
    print(f"\n[Task Planner] Decomposing: \"{user_query}\"")
    subtasks = agent.plan_task(user_query, vehicle_state=state)
    for st in subtasks:
        print(f"  Subtask {st.id} [{st.type}]: {st.instruction}")

    # ── Execute subtasks ─────────────────────────────────────────────────
    for subtask in subtasks:
        state = ros.get_vehicle_state() if use_ros else mock_state
        print(f"\n[Optimizer] Configuring MPC for subtask {subtask.id}...")
        config = agent.formulate_optimization(subtask, vehicle_state=state)

        print(f"  Target: {config.target}")
        print(f"  Gamma: {config.gamma}, V_max: {config.velocity_limit}")
        print(f"  Obstacles: {len(config.obstacles)}")

        solver.configure_from_mpc_config(config)

        t0 = time.time()
        step = 0

        while time.time() - t0 < config.timeout:
            # Get current state
            if use_ros:
                ros.spin_once()
                state = ros.get_vehicle_state()
                if state is None:
                    continue
            else:
                state = mock_state

            # Solve MPC
            u, trajectory = solver.solve(state)

            if use_ros:
                # Convert world-frame acceleration to body-frame thrust
                surge, sway = world_to_body(u[0], u[1], state[3])
                heave = float(u[2])
                yaw_thrust = float(u[3])
                ros.send_thruster_command(surge, sway, heave, yaw_thrust)
            else:
                # Mock state propagation
                mock_state = A_dyn @ mock_state + B_dyn @ u

            # Check completion
            pos_error = np.linalg.norm(state[:3] - config.target[:3])
            if step % 10 == 0:
                if use_ros:
                    surge, sway = world_to_body(u[0], u[1], state[3])
                    heave_dbg = float(u[2])
                    thrust_vals = thruster_mixing(surge, sway, heave_dbg, float(u[3]))
                    print(f"  t={time.time() - t0:.1f}s | "
                          f"pos=({state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f}) | "
                          f"err={pos_error:.3f}m | "
                          f"u_world=[{u[0]:.3f}, {u[1]:.3f}, {u[2]:.3f}, {u[3]:.3f}] | "
                          f"body=[surge={surge:.3f}, sway={sway:.3f}, heave={heave_dbg:.3f}] | "
                          f"thrusters={[f'{t:.2f}' for t in thrust_vals]}")
                else:
                    print(f"  t={time.time() - t0:.1f}s | "
                          f"pos=({state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f}) | "
                          f"err={pos_error:.3f}m | "
                          f"u=[{u[0]:.3f}, {u[1]:.3f}, {u[2]:.3f}, {u[3]:.3f}]")

            if pos_error < config.completion_threshold:
                print(f"  Subtask {subtask.id} COMPLETE (error={pos_error:.3f}m)")
                break

            step += 1
            if use_ros:
                # IPOPT solve takes ~100-500ms; only sleep the remainder
                elapsed = time.time() - t0 - step * MPC_DT
                sleep_time = MPC_DT - max(0.0, elapsed % MPC_DT)
                if sleep_time > 0.01:
                    time.sleep(sleep_time)
        else:
            print(f"  Subtask {subtask.id} TIMEOUT after {config.timeout:.0f}s "
                  f"(error={pos_error:.3f}m)")

    # ── Cleanup ──────────────────────────────────────────────────────────
    print("\nAll subtasks complete.")
    ros.shutdown()


if __name__ == "__main__":
    main()
