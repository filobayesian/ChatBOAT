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
from llm2control.config import ROBOT_START, KNOWN_OBJECTS, MPC_DT, THRUSTER_TOPIC, DAMPING_LINEAR
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
    A_dyn, B_dyn = vehicle_dynamics_matrices(MPC_DT, damping=DAMPING_LINEAR)

    if ros.is_offline:
        use_ros = False
        print("\n" + "!" * 60)
        print("rclpy not available — running in MOCK mode.")
        print("Thruster commands will NOT be published.")
        print("")
        print("To run with ROS2, use system Python (not Poetry venv):")
        print("  source /opt/ros/humble/setup.bash")
        print("  source install/setup.bash")
        print("  pip install openai numpy casadi")
        print("  cd src && python -m llm2control.main")
        print("!" * 60 + "\n")
    else:
        # Wait for first odometry (give simulator time to start)
        print("Waiting for odometry...")
        for i in range(150):  # 15 seconds
            ros.spin_once(timeout_sec=0.1)
            if ros.get_vehicle_state() is not None:
                print(f"Odometry received after {(i + 1) * 0.1:.1f}s")
                break
        else:
            print("\n" + "!" * 60)
            print("rclpy OK but no odometry after 15s — running in MOCK mode.")
            print("Thruster commands will NOT be published!")
            print("Check: is the simulator running? Is /chatboat/odometry being published?")
            print("!" * 60 + "\n")

        use_ros = ros.get_vehicle_state() is not None
    mock_state = np.zeros(10)
    mock_state[:3] = ROBOT_START  # x, y, z; phi=0, psi=0, all velocities=0
    if not use_ros:
        print(f"Mock start state: {mock_state[:5]}")
    else:
        print(f"ROS mode active — publishing to {THRUSTER_TOPIC}")

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
        if subtask.type == "unsupported":
            print(f"\n  Subtask {subtask.id} UNSUPPORTED: {subtask.instruction}")
            print(f"  Skipping — this task requires capabilities beyond the current MPC primitives.")
            continue

        state = ros.get_vehicle_state() if use_ros else mock_state
        print(f"\n[Optimizer] Configuring MPC for subtask {subtask.id}...")
        try:
            config = agent.formulate_optimization(subtask, vehicle_state=state)
        except ValueError as e:
            print(f"  Subtask {subtask.id} REJECTED: {e}")
            continue

        print(f"  Target: {config.target}")
        print(f"  Gamma: {config.gamma}, V_max: {config.velocity_limit}")
        print(f"  Obstacles: {len(config.obstacles)}")

        solver.configure_from_mpc_config(config)

        t0 = time.time()
        step = 0
        max_steps = int(config.timeout / MPC_DT)

        while (step < max_steps) if not use_ros else (time.time() - t0 < config.timeout):
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
                # Use the SAME state that was passed to solver.solve()
                # for the body-frame conversion — avoids yaw mismatch
                surge, sway = world_to_body(u[0], u[1], state[4])  # psi at index 4
                heave = float(u[2])
                roll_thrust = float(u[3])
                yaw_thrust = float(u[4])
                ros.send_thruster_command(surge, sway, heave, roll_thrust, yaw_thrust)
            else:
                # Mock state propagation
                mock_state = A_dyn @ mock_state + B_dyn @ u

            # Check completion
            pos_error = np.linalg.norm(state[:3] - config.target[:3])
            if step % 10 == 0:
                if use_ros:
                    surge, sway = world_to_body(u[0], u[1], state[4])
                    heave_dbg = float(u[2])
                    roll_dbg = float(u[3])
                    yaw_dbg = float(u[4])
                    thrust_vals = thruster_mixing(surge, sway, heave_dbg, roll_dbg, yaw_dbg)
                    print(f"  t={time.time() - t0:.1f}s | "
                          f"pos=({state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f}) | "
                          f"err={pos_error:.3f}m | "
                          f"u_world=[{u[0]:.3f}, {u[1]:.3f}, {u[2]:.3f}, {u[3]:.3f}, {u[4]:.3f}] | "
                          f"body=[surge={surge:.3f}, sway={sway:.3f}, heave={heave_dbg:.3f}] | "
                          f"thrusters={[f'{t:.2f}' for t in thrust_vals]}")
                else:
                    print(f"  t={step * MPC_DT:.1f}s | "
                          f"pos=({state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f}) | "
                          f"err={pos_error:.3f}m | "
                          f"u=[{u[0]:.3f}, {u[1]:.3f}, {u[2]:.3f}, {u[3]:.3f}, {u[4]:.3f}]")

            if pos_error < config.completion_threshold:
                print(f"  Subtask {subtask.id} COMPLETE (error={pos_error:.3f}m)")
                break

            step += 1
            if use_ros:
                next_time = t0 + (step + 1) * MPC_DT
                sleep_time = next_time - time.time()
                if sleep_time > 0.001:
                    time.sleep(sleep_time)
        else:
            print(f"  Subtask {subtask.id} TIMEOUT after {config.timeout:.0f}s "
                  f"(error={pos_error:.3f}m)")

    # ── Cleanup ──────────────────────────────────────────────────────────
    print("\nAll subtasks complete.")
    ros.shutdown()


if __name__ == "__main__":
    main()
