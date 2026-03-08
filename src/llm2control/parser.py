"""Validation and parsing of LLM outputs into structured dataclasses."""

from dataclasses import dataclass, field

import numpy as np

from llm2control.config import VALID_PROBLEM_TYPES, PROBLEM_TYPE_GENERAL


@dataclass
class Subtask:
    """A single atomic subtask from the Task Planner."""
    id: int
    instruction: str
    type: str  # navigate | hold_position | return_home


@dataclass
class MPCConfig:
    """MPC configuration from the Optimization Formulator."""
    target: np.ndarray          # [x, y, z, psi]
    Q: np.ndarray               # 10x10 diagonal
    R: np.ndarray               # 5x5 diagonal
    gamma: float                # CBF safety parameter (0, 1]
    problem_type: str = "general"  # general | descent_ascent | lateral
    obstacles: list[dict] = field(default_factory=list)
    velocity_limit: float = 0.5
    completion_threshold: float = 0.15
    timeout: float = 30.0


def parse_subtasks(raw: dict) -> list[Subtask]:
    """Parse Task Planner JSON output into Subtask list.

    Parameters
    ----------
    raw : dict with key "subtasks" containing list of subtask dicts

    Returns
    -------
    list[Subtask]

    Raises
    ------
    ValueError if validation fails
    """
    subtask_list = raw.get("subtasks")
    if not subtask_list:
        raise ValueError("Task planner returned empty or missing 'subtasks'")

    valid_types = {"navigate", "hold_position", "return_home", "unsupported"}
    result = []
    for s in subtask_list:
        st = Subtask(
            id=int(s["id"]),
            instruction=str(s["instruction"]),
            type=str(s["type"]),
        )
        if st.type not in valid_types:
            raise ValueError(f"Unknown subtask type '{st.type}' in subtask {st.id}")
        result.append(st)

    return result


def parse_mpc_config(raw: dict) -> MPCConfig:
    """Parse Optimization Formulator JSON output into MPCConfig.

    Parameters
    ----------
    raw : dict with keys vehicle_target, weights, gamma, obstacles, etc.

    Returns
    -------
    MPCConfig

    Raises
    ------
    ValueError if validation fails
    """
    # Problem type
    problem_type = str(raw.get("problem_type", PROBLEM_TYPE_GENERAL))
    if problem_type not in VALID_PROBLEM_TYPES:
        raise ValueError(
            f"Unknown problem_type '{problem_type}', "
            f"must be one of {sorted(VALID_PROBLEM_TYPES)}"
        )

    # Target
    target = np.array(raw["vehicle_target"], dtype=float)
    if target.shape != (4,):
        raise ValueError(f"vehicle_target must have 4 elements, got {target.shape}")

    # Weights -> build diagonal matrices (10D state, 5D control)
    w = raw["weights"]
    Q_pos = float(w["Q_pos"])
    Q_yaw = float(w["Q_yaw"])
    Q_vel = float(w["Q_vel"])
    R_lin = float(w["R_lin"])
    R_rot = float(w["R_rot"])

    # Q_roll: default 0.0 for general (backwards-compatible), expected for
    # descent_ascent / lateral where roll stabilization matters.
    Q_roll = float(w.get("Q_roll", 0.0))

    # 10x10: [x, y, z, roll, yaw, vx, vy, vz, v_roll, v_yaw]
    Q = np.diag([Q_pos, Q_pos, Q_pos, Q_roll, Q_yaw,
                 Q_vel, Q_vel, Q_vel, Q_vel, Q_vel])

    # 5x5: [u_x, u_y, u_z, u_roll, u_yaw]
    R = np.diag([R_lin, R_lin, R_lin, R_rot, R_rot])

    # Gamma
    gamma = float(raw["gamma"])
    if not (0 < gamma <= 1.0):
        raise ValueError(f"gamma must be in (0, 1], got {gamma}")

    # Obstacles
    obstacles = []
    for obs in raw.get("obstacles", []):
        obstacles.append({
            "name": str(obs["name"]),
            "position": [float(v) for v in obs["position"]],
            "radius": float(obs["radius"]),
        })

    return MPCConfig(
        target=target,
        Q=Q,
        R=R,
        gamma=gamma,
        problem_type=problem_type,
        obstacles=obstacles,
        velocity_limit=float(raw.get("velocity_limit", 0.5)),
        completion_threshold=float(raw.get("completion_threshold", 0.15)),
        timeout=float(raw.get("timeout", 30.0)),
    )
