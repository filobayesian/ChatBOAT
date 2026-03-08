"""10-DOF vehicle model and thruster mixing for BlueROV2 Heavy."""

import numpy as np


def vehicle_dynamics_matrices(dt: float):
    """Return (A, B) for the discrete 10D double-integrator.

    State x = [x, y, z, phi, psi, dx, dy, dz, dphi, dpsi]
    Control u = [u_x, u_y, u_z, u_phi, u_psi]   (world-frame accelerations)

    x_{k+1} = A @ x_k + B @ u_k
    """
    I5 = np.eye(5)
    Z5 = np.zeros((5, 5))

    A = np.block([
        [I5, I5 * dt],
        [Z5, I5],
    ])
    B = np.block([
        [0.5 * I5 * dt**2],
        [I5 * dt],
    ])
    return A, B


def vehicle_dynamics_matrices_8d(dt: float):
    """Return (A, B) for the old 8D double-integrator (backward compat).

    State x = [x, y, z, psi, dx, dy, dz, dpsi]
    Control u = [u_x, u_y, u_z, u_psi]
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


def thruster_mixing(surge: float, sway: float, heave: float, roll: float, yaw: float) -> list[float]:
    """BlueROV2 Heavy 8-thruster mixing.

    Horizontal thrusters at +/-45 deg angles:
      T1 FrontRight, T2 FrontLeft, T3 BackRight, T4 BackLeft
    Vertical thrusters (differential for roll):
      T5 port-front, T6 starboard-front, T7 port-back, T8 starboard-back

    Returns list of 8 values in [-1, 1].
    """
    t1 = surge + sway + yaw   # FrontRight
    t2 = surge - sway - yaw   # FrontLeft
    t3 = -surge + sway - yaw  # BackRight
    t4 = -surge - sway + yaw  # BackLeft
    t5 = heave + roll          # port-front
    t6 = heave - roll          # starboard-front
    t7 = heave + roll          # port-back
    t8 = heave - roll          # starboard-back

    return [max(-1.0, min(1.0, t)) for t in [t1, t2, t3, t4, t5, t6, t7, t8]]
