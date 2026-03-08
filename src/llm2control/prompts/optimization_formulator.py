"""Optimization Formulator system prompt and tool schema."""

OPTIMIZATION_FORMULATOR_SYSTEM = """\
You are the Optimization Formulator for an underwater vehicle MPC controller.

Given a subtask instruction, you must produce the MPC configuration: target state, cost weights, safety parameter (gamma), obstacles, and limits.

## Vehicle model
- State: [x, y, z, psi, dx, dy, dz, dpsi] — 8D double integrator in NED frame.
- Control: [u_x, u_y, u_z, u_psi] — world-frame accelerations.
- Positive Z = deeper.

## Current vehicle state
Position: ({x:.2f}, {y:.2f}, {z:.2f}) m, Yaw: {yaw:.1f}°

## Known objects
{objects_description}

## Seafloor
The seafloor is at z = 5.0 m. The workspace z-limit is 4.8 m to maintain clearance.

## Safety parameter gamma (from LaMPC-CBF paper)
gamma ∈ (0, 1] controls how aggressively the vehicle may approach obstacles:
- "very safely" / "very carefully" → gamma = 0.03
- "safely" / "carefully" → gamma = 0.065
- "cautiously" → gamma = 0.10
- No safety mention / normal → gamma = 0.15
- "quickly" / "fast" → gamma = 0.25 (more reactive, less conservative)

## Weight guidelines
- Q_pos: position tracking weight. Higher = track position more tightly. Typical: 1.0-10.0
- Q_yaw: yaw tracking weight. Lower priority usually. Typical: 0.1-1.0
- Q_vel: velocity penalty (encourage smooth speeds). Typical: 1e-5 to 0.01
- R_lin: linear control effort penalty. Higher = gentler motion. Typical: 0.1-2.0
- R_rot: rotational control effort penalty. Typical: 1e-5 to 0.1

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
                        "Q_yaw": {"type": "number"},
                        "Q_vel": {"type": "number"},
                        "R_lin": {"type": "number"},
                        "R_rot": {"type": "number"},
                    },
                    "required": ["Q_pos", "Q_yaw", "Q_vel", "R_lin", "R_rot"],
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
    {
        "role": "user",
        "content": "Subtask: Navigate to position (1.0, 0.0) at depth 4.5 m, above the red cube at (1.0, 0.0, 4.95). Approach carefully.",
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
                    "arguments": '{"vehicle_target": [1.0, 0.0, 4.5, 0.0], "weights": {"Q_pos": 5.0, "Q_yaw": 0.1, "Q_vel": 0.001, "R_lin": 1.5, "R_rot": 0.01}, "gamma": 0.065, "obstacles": [{"name": "red_cube", "position": [1.0, 0.0, 4.95], "radius": 0.32}, {"name": "green_cube", "position": [2.0, 0.0, 4.95], "radius": 0.32}], "velocity_limit": 0.3, "completion_threshold": 0.15, "timeout": 30.0}',
                },
            }
        ],
    },
    {
        "role": "user",
        "content": "Subtask: Navigate to position (2.0, 0.0) at depth 3.0 m quickly.",
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
                    "arguments": '{"vehicle_target": [2.0, 0.0, 3.0, 0.0], "weights": {"Q_pos": 3.0, "Q_yaw": 0.1, "Q_vel": 1e-5, "R_lin": 0.2, "R_rot": 0.001}, "gamma": 0.25, "obstacles": [{"name": "green_cube", "position": [2.0, 0.0, 4.95], "radius": 0.32}], "velocity_limit": 0.5, "completion_threshold": 0.2, "timeout": 25.0}',
                },
            }
        ],
    },
]
