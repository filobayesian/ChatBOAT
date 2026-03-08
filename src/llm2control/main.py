"""LLM-MPC pipeline orchestrator.

Usage:
  OPENROUTER_API_KEY=... python -m llm2control.main
  OPENROUTER_API_KEY=... python -m llm2control.main --dashboard [--port 8765]

Modes:
  - With ROS2 (rclpy available): reads odometry, publishes thruster commands
  - Without ROS2: uses mock state propagation for offline testing
  - With --dashboard: starts WebSocket server, accepts queries from browser
"""

import argparse
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
    parser = argparse.ArgumentParser(description="LLM-MPC pipeline orchestrator")
    parser.add_argument("--dashboard", action="store_true",
                        help="Start WebSocket server for browser dashboard")
    parser.add_argument("--port", type=int, default=8765,
                        help="WebSocket port (default 8765)")
    args = parser.parse_args()

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

    # ── Optional dashboard bridge ────────────────────────────────────────
    dashboard = None
    if args.dashboard:
        from llm2control.dashboard_bridge import DashboardBridge
        dashboard = DashboardBridge(port=args.port)
        dashboard.start()

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

    # ── Main loop (repeats in dashboard mode) ────────────────────────────
    while True:
        # ── User input ───────────────────────────────────────────────────
        print("\n" + "=" * 60)
        print(_build_scene_description(ros.get_vehicle_state()))
        print("=" * 60)

        if dashboard:
            dashboard.broadcast("status", {"phase": "idle"})
            print("\n[Dashboard] Waiting for query from browser...")
            user_query = dashboard.wait_for_query()
            print(f"[Dashboard] Received query: \"{user_query}\"")
        else:
            user_query = input("\nEnter navigation command: ").strip()

        if not user_query:
            print("No command entered.")
            if not dashboard:
                break
            continue

        # ── Task Planning ────────────────────────────────────────────────
        state = ros.get_vehicle_state() if use_ros else mock_state
        print(f"\n[Task Planner] Decomposing: \"{user_query}\"")
        if dashboard:
            dashboard.broadcast("status", {"phase": "planning"})

        subtasks = agent.plan_task(user_query, vehicle_state=state)
        for st in subtasks:
            print(f"  Subtask {st.id} [{st.type}]: {st.instruction}")

        if dashboard:
            dashboard.broadcast("subtasks", {
                "subtasks": [
                    {"id": st.id, "instruction": st.instruction, "type": st.type}
                    for st in subtasks
                ],
                "tokens": agent.last_usage,
            })

        # ── Execute subtasks ─────────────────────────────────────────────
        for subtask in subtasks:
            if subtask.type == "unsupported":
                print(f"\n  Subtask {subtask.id} UNSUPPORTED: {subtask.instruction}")
                print(f"  Skipping — this task requires capabilities beyond the current MPC primitives.")
                if dashboard:
                    dashboard.broadcast("subtask_skipped", {
                        "id": subtask.id,
                        "instruction": subtask.instruction,
                    })
                continue

            if dashboard:
                dashboard.broadcast("subtask_active", {"id": subtask.id})

            state = ros.get_vehicle_state() if use_ros else mock_state
            print(f"\n[Optimizer] Configuring MPC for subtask {subtask.id}...")
            try:
                config = agent.formulate_optimization(subtask, vehicle_state=state)
            except ValueError as e:
                print(f"  Subtask {subtask.id} REJECTED: {e}")
                if dashboard:
                    dashboard.broadcast("mpc_config_error", {
                        "id": subtask.id,
                        "error": str(e),
                    })
                continue

            print(f"  Target: {config.target}")
            print(f"  Gamma: {config.gamma}, V_max: {config.velocity_limit}")
            print(f"  Obstacles: {len(config.obstacles)}")

            if dashboard:
                # Extract weight names from Q/R diagonals for display
                Q_diag = np.diag(config.Q)
                R_diag = np.diag(config.R)
                dashboard.broadcast("mpc_config", {
                    "id": subtask.id,
                    "target": config.target.tolist(),
                    "weights": {
                        "Q_pos": float(Q_diag[0]),
                        "Q_roll": float(Q_diag[3]),
                        "Q_yaw": float(Q_diag[4]),
                        "Q_vel": float(Q_diag[5]),
                        "R_lin": float(R_diag[0]),
                        "R_rot": float(R_diag[3]),
                    },
                    "gamma": config.gamma,
                    "obstacles": config.obstacles,
                    "velocity_limit": config.velocity_limit,
                    "completion_threshold": config.completion_threshold,
                    "timeout": config.timeout,
                    "tokens": agent.last_usage,
                })

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

                # Compute body-frame thrusters (always, for dashboard + logging)
                surge, sway = world_to_body(u[0], u[1], state[4])
                heave = float(u[2])
                roll_thrust = float(u[3])
                yaw_thrust = float(u[4])
                thrust_vals = thruster_mixing(surge, sway, heave, roll_thrust, yaw_thrust)

                if use_ros:
                    ros.spin_once()  # process callbacks that arrived during solve
                    fresh = ros.get_vehicle_state()
                    if fresh is not None:
                        state = fresh
                    ros.send_thruster_command(surge, sway, heave, roll_thrust, yaw_thrust)
                else:
                    # Mock state propagation
                    mock_state = A_dyn @ mock_state + B_dyn @ u

                # Check completion
                pos_error = np.linalg.norm(state[:3] - config.target[:3])
                elapsed = time.time() - t0 if use_ros else step * MPC_DT

                # Dashboard update (every step)
                if dashboard:
                    predicted = trajectory[:, :3].tolist() if trajectory is not None else []
                    dashboard.broadcast("control_update", {
                        "state": state.tolist(),
                        "control": u.tolist() if hasattr(u, 'tolist') else list(u),
                        "thrusters": thrust_vals,
                        "error": float(pos_error),
                        "elapsed": float(elapsed),
                        "predicted": predicted,
                    })

                # Console logging
                if step % 10 == 0:
                    if use_ros:
                        print(f"  t={elapsed:.1f}s | "
                              f"pos=({state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f}) | "
                              f"err={pos_error:.3f}m | "
                              f"u_world=[{u[0]:.3f}, {u[1]:.3f}, {u[2]:.3f}, {u[3]:.3f}, {u[4]:.3f}] | "
                              f"body=[surge={surge:.3f}, sway={sway:.3f}, heave={heave:.3f}] | "
                              f"thrusters={[f'{t:.2f}' for t in thrust_vals]}")
                    else:
                        print(f"  t={step * MPC_DT:.1f}s | "
                              f"pos=({state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f}) | "
                              f"err={pos_error:.3f}m | "
                              f"u=[{u[0]:.3f}, {u[1]:.3f}, {u[2]:.3f}, {u[3]:.3f}, {u[4]:.3f}]")

                if pos_error < config.completion_threshold:
                    print(f"  Subtask {subtask.id} COMPLETE (error={pos_error:.3f}m)")
                    if dashboard:
                        dashboard.broadcast("subtask_complete", {
                            "id": subtask.id,
                            "error": float(pos_error),
                        })
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
                if dashboard:
                    dashboard.broadcast("subtask_complete", {
                        "id": subtask.id,
                        "error": float(pos_error),
                        "timeout": True,
                    })

        # ── All subtasks done ────────────────────────────────────────────
        print("\nAll subtasks complete.")
        if dashboard:
            dashboard.broadcast("all_complete", {})
            continue  # loop back for next query
        else:
            break  # exit in non-dashboard mode

    # ── Cleanup ──────────────────────────────────────────────────────────
    if dashboard:
        dashboard.stop()
    ros.shutdown()


if __name__ == "__main__":
    main()
