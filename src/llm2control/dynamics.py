"""8-DOF vehicle model and thruster mixing for BlueROV2 Heavy."""

import numpy as np


def vehicle_dynamics_matrices(dt: float):
    """Return (A, B) for the discrete 8D double-integrator.

    State x = [x, y, z, psi, dx, dy, dz, dpsi]
    Control u = [u_x, u_y, u_z, u_psi]   (world-frame accelerations)

    x_{k+1} = A @ x_k + B @ u_k
    """
    I4 = np.eye(4)
    Z4 = np.zeros((4, 4))

    A = np.block([
        [I4, I4 * dt],
        [Z4, I4],
    ])
    B = np.block([
        [0.5 * I4 * dt**2],
        [I4 * dt],
    ])
    return A, B


def world_to_body(u_x: float, u_y: float, psi: float):
    """Convert world-frame XY acceleration to body-frame surge/sway.

    Parameters
    ----------
    u_x, u_y : world-frame accelerations
    psi : yaw angle (radians)

    Returns
    -------
    surge, sway : body-frame accelerations
    """
    cos_psi = np.cos(psi)
    sin_psi = np.sin(psi)
    surge = u_x * cos_psi + u_y * sin_psi
    sway = -u_x * sin_psi + u_y * cos_psi
    return float(surge), float(sway)


def thruster_mixing(surge: float, sway: float, heave: float, yaw: float) -> list[float]:
    """BlueROV2 Heavy 8-thruster mixing.

    Horizontal thrusters at ±45° angles:
      T1 FrontRight, T2 FrontLeft, T3 BackRight, T4 BackLeft
    Vertical thrusters (equal for pure heave):
      T5-T8

    Returns list of 8 values in [-1, 1].
    """
    t1 = surge + sway + yaw   # FrontRight
    t2 = surge - sway - yaw   # FrontLeft
    t3 = -surge + sway - yaw  # BackRight
    t4 = -surge - sway + yaw  # BackLeft
    t5 = heave
    t6 = heave
    t7 = heave
    t8 = heave

    return [max(-1.0, min(1.0, t)) for t in [t1, t2, t3, t4, t5, t6, t7, t8]]
