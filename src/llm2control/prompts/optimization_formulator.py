"""Optimization Formulator system prompt, tool schema, and few-shot examples."""

OPTIMIZATION_FORMULATOR_SYSTEM = """\
You are the Optimization Formulator for an underwater vehicle MPC controller.

Given a subtask instruction, you must produce the MPC configuration: \
target state, cost weights, safety parameter (gamma), obstacles, and limits.

## Environment — Simulated Ocean (NED frame)
- Coordinate frame: NED (North-East-Down). Positive Z = deeper.
- Water surface: z = 0.0 m
- Seafloor: z = 5.0 m (flat, 100×100 m box)
- Water column depth: 5.0 m
- Workspace bounds: x ∈ [-10, 10], y ∈ [-10, 10], z ∈ [-0.5, 4.8]
- z = -0.5 allows surface operation; z = 4.8 provides 0.2 m seafloor clearance
- Robot default spawn position: (0.0, 0.0, 2.0) — roughly mid-water column

## Known objects in the scene
{objects_description}
All objects sit on the seafloor (z ≈ 4.95 m). The cubes are 0.1 m side length.
To position above an object, use z ≈ 4.5 m (0.45 m above seafloor, 0.3 m above cube top).

## Vehicle model
- BlueROV2 Heavy with 8 thrusters (6 DOF: surge, sway, heave, roll, pitch, yaw)
- State: [x, y, z, phi, psi, dx, dy, dz, dphi, dpsi] — 10D double integrator with linear damping (drag) in NED frame
  - phi = roll angle (rad), psi = yaw angle (rad)
- Control: [u_x, u_y, u_z, u_phi, u_psi] — 5D world-frame accelerations, all always active
- The vehicle is neutrally buoyant but tends to develop roll oscillations without active stabilization
- Roll stabilisation is handled by the Q_roll weight — no need to restrict control axes

## Current vehicle state
Position: ({x:.2f}, {y:.2f}, {z:.2f}) m, Yaw: {yaw:.1f}°

## Unsupported tasks
If the subtask requires capabilities beyond simple navigation (e.g., manipulation, grasping, \
dynamic trajectory tracking, formation control), the task is UNSUPPORTED. Do not attempt to \
formulate an MPC config — instead set the target to [0,0,0,0] and note it in the response.

## Safety parameter gamma (from LaMPC-CBF paper)
gamma ∈ (0, 1] controls how aggressively the vehicle may approach obstacles:
- "very safely" / "very carefully" → gamma = 0.03
- "safely" / "carefully" → gamma = 0.065
- "cautiously" → gamma = 0.10
- No safety mention / normal → gamma = 0.15
- "quickly" / "fast" → gamma = 0.25 (more reactive, less conservative)

## Weight guidelines
- Q_pos: position tracking weight (x, y, z). Higher = track position tightly. Typical: 1.0-10.0
- Q_roll: roll stabilization weight (phi → 0). CRITICAL for stability. Typical: 3.0-8.0
  - Use higher values (5.0-8.0) when near obstacles or the seafloor
  - Use moderate values (3.0-5.0) for open-water navigation
- Q_yaw: yaw tracking weight. Typical: 0.1-1.0
- Q_vel: velocity penalty. Typical: 1e-5 to 0.01
- R_lin: linear control effort penalty. Typical: 0.1-2.0
- R_rot: rotational control effort penalty (for both roll and yaw control). Typical: 0.01-0.5

For "careful" or "slow" commands: increase R_lin (1.0-2.0), decrease velocity_limit.
For "fast" or "quick" commands: decrease R_lin (0.1-0.3), increase velocity_limit.

## Rules
1. Always include obstacles that are near the planned path (within 2m).
2. The vehicle_target must be [x, y, z, yaw]. If yaw is not specified, use 0.0.
3. Obstacle radius should include vehicle collision radius (0.25m) as padding: object_radius + 0.25.
4. For "hold_position" subtasks, set the target to the current position and use high Q_pos.
5. For "return_home" subtasks, set target to [0.0, 0.0, 2.0, 0.0].
6. completion_threshold: typically 0.15-0.3 m. Use smaller for precision tasks.
7. timeout: typically 20-60 seconds depending on distance.
"""

OPTIMIZATION_FORMULATOR_TOOL = {
    "type": "function",
    "function": {
        "name": "configure_mpc",
        "description": "Output MPC configuration for one subtask.",
        "parameters": {
            "type": "object",
            "properties": {
                "vehicle_target": {
                    "type": "array",
                    "items": {"type": "number"},
                    "minItems": 4,
                    "maxItems": 4,
                    "description": "[x, y, z, yaw] target state",
                },
                "weights": {
                    "type": "object",
                    "properties": {
                        "Q_pos": {"type": "number"},
                        "Q_roll": {"type": "number"},
                        "Q_yaw": {"type": "number"},
                        "Q_vel": {"type": "number"},
                        "R_lin": {"type": "number"},
                        "R_rot": {"type": "number"},
                    },
                    "required": ["Q_pos", "Q_roll", "Q_yaw", "Q_vel", "R_lin", "R_rot"],
                },
                "gamma": {
                    "type": "number",
                    "description": "CBF safety parameter in (0, 1]",
                },
                "obstacles": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "name": {"type": "string"},
                            "position": {
                                "type": "array",
                                "items": {"type": "number"},
                                "minItems": 3,
                                "maxItems": 3,
                            },
                            "radius": {"type": "number"},
                        },
                        "required": ["name", "position", "radius"],
                    },
                },
                "velocity_limit": {
                    "type": "number",
                    "description": "Max velocity per axis (m/s)",
                },
                "completion_threshold": {
                    "type": "number",
                    "description": "Position error (m) to consider subtask complete",
                },
                "timeout": {
                    "type": "number",
                    "description": "Max time (s) for this subtask",
                },
            },
            "required": [
                "vehicle_target",
                "weights",
                "gamma",
                "obstacles",
                "velocity_limit",
                "completion_threshold",
                "timeout",
            ],
        },
    },
}

# ── Few-shot examples ────────────────────────────────────────────────────────

OPTIMIZATION_FORMULATOR_EXAMPLES = [
    # Example 1: navigate to a point near an object
    {
        "role": "user",
        "content": (
            "Subtask: Navigate to position (1.0, 0.0) at depth 4.5 m, "
            "above the red cube at (1.0, 0.0, 4.95)."
        ),
    },
    {
        "role": "assistant",
        "content": None,
        "tool_calls": [
            {
                "id": "of_ex1",
                "type": "function",
                "function": {
                    "name": "configure_mpc",
                    "arguments": (
                        '{"vehicle_target": [1.0, 0.0, 4.5, 0.0], '
                        '"weights": {"Q_pos": 5.0, "Q_roll": 6.0, "Q_yaw": 0.3, '
                        '"Q_vel": 0.001, "R_lin": 1.0, "R_rot": 0.1}, '
                        '"gamma": 0.065, '
                        '"obstacles": [{"name": "red_cube", "position": [1.0, 0.0, 4.95], '
                        '"radius": 0.32}], '
                        '"velocity_limit": 0.4, '
                        '"completion_threshold": 0.15, '
                        '"timeout": 30.0}'
                    ),
                },
            }
        ],
    },
    # Example 2: quick horizontal movement
    {
        "role": "user",
        "content": (
            "Subtask: Navigate to position (2.0, 0.0) at depth 3.0 m quickly."
        ),
    },
    {
        "role": "assistant",
        "content": None,
        "tool_calls": [
            {
                "id": "of_ex2",
                "type": "function",
                "function": {
                    "name": "configure_mpc",
                    "arguments": (
                        '{"vehicle_target": [2.0, 0.0, 3.0, 0.0], '
                        '"weights": {"Q_pos": 3.0, "Q_roll": 4.0, "Q_yaw": 0.5, '
                        '"Q_vel": 1e-5, "R_lin": 0.2, "R_rot": 0.05}, '
                        '"gamma": 0.25, '
                        '"obstacles": [], '
                        '"velocity_limit": 0.5, '
                        '"completion_threshold": 0.2, '
                        '"timeout": 25.0}'
                    ),
                },
            }
        ],
    },
    # Example 3: return home
    {
        "role": "user",
        "content": "Subtask: Return to start position (0.0, 0.0) at depth 2.0 m.",
    },
    {
        "role": "assistant",
        "content": None,
        "tool_calls": [
            {
                "id": "of_ex3",
                "type": "function",
                "function": {
                    "name": "configure_mpc",
                    "arguments": (
                        '{"vehicle_target": [0.0, 0.0, 2.0, 0.0], '
                        '"weights": {"Q_pos": 3.0, "Q_roll": 5.0, "Q_yaw": 0.3, '
                        '"Q_vel": 0.001, "R_lin": 1.0, "R_rot": 0.1}, '
                        '"gamma": 0.15, '
                        '"obstacles": [], '
                        '"velocity_limit": 0.4, '
                        '"completion_threshold": 0.2, '
                        '"timeout": 30.0}'
                    ),
                },
            }
        ],
    },
]
