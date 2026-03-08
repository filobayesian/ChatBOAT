"""Robot constants, topic names, workspace bounds, and MPC defaults."""

import numpy as np

# ── ROS2 Topics ──────────────────────────────────────────────────────────────
THRUSTER_TOPIC = "/chatboat/thruster_commands"
ODOMETRY_TOPIC = "/chatboat/odometry"

# ── Scene ────────────────────────────────────────────────────────────────────
CUBE_A_POS = np.array([1.0, 0.0, 4.95])  # Red cube (NED frame)
CUBE_B_POS = np.array([2.0, 0.0, 4.95])  # Green cube
CUBE_RADIUS = 0.07  # half-diagonal of 0.1m cube ≈ 0.07m bounding sphere
SURFACE_Z = 0.0    # water surface in NED
SEAFLOOR_Z = 5.0   # top of the seafloor box
WATER_DEPTH = 5.0   # total water column depth

ROBOT_START = np.array([0.0, 0.0, 2.0])  # default spawn position (NED)

# ── Workspace bounds ────────────────────────────────────────────────────────
WS_X_MIN, WS_X_MAX = -10.0, 10.0
WS_Y_MIN, WS_Y_MAX = -10.0, 10.0
WS_Z_MIN, WS_Z_MAX = -0.5, 4.8  # allow surface operation, above seafloor

# ── Vehicle geometry ─────────────────────────────────────────────────────────
COLLISION_RADIUS = 0.25  # BlueROV2 bounding sphere (approximate)

# ── MPC defaults ─────────────────────────────────────────────────────────────
MPC_DT = 0.1
MPC_HORIZON = 15
V_MAX_DEFAULT = 0.5  # m/s per axis
U_MAX_DEFAULT = 0.3  # acceleration limit (thrust fraction-ish)

# ── Linear damping coefficients (normalized: deceleration per unit velocity) ─
# v_{k+1} = (1 - d_i * dt) * v_k + dt * u_k
# Conservative initial estimates for BlueROV2 Heavy
DAMPING_LINEAR = np.array([
    1.0,   # d_x  (surge)
    1.0,   # d_y  (sway)
    1.5,   # d_z  (heave — larger projected area)
    0.3,   # d_phi (roll)
    0.3,   # d_psi (yaw)
])

# ── Thrust scaling ──────────────────────────────────────────────────────────
THRUST_SCALE = 50.0  # Stonefish setpoints: mixing [-1,1] × scale → [-50,50]

# ── Known objects (for scene description sent to LLM) ────────────────────────
KNOWN_OBJECTS = [
    {"name": "red_cube", "position": CUBE_A_POS.tolist(), "radius": CUBE_RADIUS},
    {"name": "green_cube", "position": CUBE_B_POS.tolist(), "radius": CUBE_RADIUS},
]
