"""10D CasADi MPC with CBF safety constraints for BlueROV2 navigation.

Inspired by LaMPC-CBF (Song et al.): quadratic tracking cost + CBF obstacle
avoidance constraints.

State x = [x, y, z, phi, psi, dx, dy, dz, dphi, dpsi]  (10D)
Control u = [u_x, u_y, u_z, u_phi, u_psi]               (5D, world-frame accelerations)

Problem types:
  - "general"        : full 10D, all 5 controls active (default/legacy)
  - "descent_ascent" : only u_z and u_phi active (vertical + roll stabilisation)
  - "lateral"        : u_x, u_y, u_phi, u_psi active (horizontal + orientation)
"""

import numpy as np

try:
    import casadi as ca
except ImportError:
    ca = None
    print("[mpc] CasADi not installed — MPC solver unavailable")

from llm2control.config import (
    MPC_DT, MPC_HORIZON,
    WS_X_MIN, WS_X_MAX,
    WS_Y_MIN, WS_Y_MAX,
    WS_Z_MIN, WS_Z_MAX,
    V_MAX_DEFAULT, U_MAX_DEFAULT,
    COLLISION_RADIUS,
)

VALID_PROBLEM_TYPES = {"general", "descent_ascent", "lateral"}


class VehicleMPCSolver:
    """Receding-horizon MPC for BlueROV2 vehicle navigation.

    Uses CasADi Opti (NLP / IPOPT) with:
    - Quadratic tracking cost (position + roll + yaw + velocity regularisation)
    - CBF obstacle-avoidance constraints
    - Workspace & actuation bounds
    - Problem-type-specific control constraints
    """

    def __init__(self, dt: float = MPC_DT, horizon: int = MPC_HORIZON):
        if ca is None:
            raise RuntimeError("CasADi is required for VehicleMPCSolver")
        self.dt = dt
        self.N = horizon

        # Defaults (overwritten by configure())
        self.target = np.zeros(4)       # [x, y, z, psi]
        self.Q = np.diag([1.0] * 10)
        self.R = np.diag([0.5] * 5)
        self.gamma = 0.15
        self.obstacles: list[dict] = []
        self.v_max = V_MAX_DEFAULT
        self.u_max = U_MAX_DEFAULT
        self.lam_vel = 0.001  # velocity regularisation weight
        self.problem_type = "general"

        # Warm-start storage
        self._prev_x: np.ndarray | None = None
        self._prev_u: np.ndarray | None = None

    # ── Configuration ────────────────────────────────────────────────────────

    def configure(self, target: np.ndarray, Q: np.ndarray, R: np.ndarray,
                  gamma: float, obstacles: list[dict],
                  v_max: float = V_MAX_DEFAULT,
                  u_max: float = U_MAX_DEFAULT,
                  problem_type: str = "general"):
        """Set MPC parameters (called once per subtask)."""
        self.target = np.asarray(target, dtype=float)
        self.Q = np.asarray(Q, dtype=float)
        self.R = np.asarray(R, dtype=float)
        self.gamma = float(gamma)
        self.obstacles = obstacles
        self.v_max = float(v_max)
        self.u_max = float(u_max)
        if problem_type not in VALID_PROBLEM_TYPES:
            raise ValueError(
                f"Unknown problem_type '{problem_type}'. "
                f"Must be one of {VALID_PROBLEM_TYPES}"
            )
        self.problem_type = problem_type
        # Reset warm start on new subtask
        self._prev_x = None
        self._prev_u = None

    def configure_from_mpc_config(self, config: "MPCConfig"):
        """Configure from a parsed MPCConfig dataclass."""
        self.configure(
            target=config.target,
            Q=config.Q,
            R=config.R,
            gamma=config.gamma,
            obstacles=config.obstacles,
            v_max=config.velocity_limit,
            u_max=U_MAX_DEFAULT,
            problem_type=getattr(config, "problem_type", "general"),
        )

    def update_gamma(self, new_gamma: float):
        self.gamma = float(new_gamma)

    # ── Solver ───────────────────────────────────────────────────────────────

    def solve(self, current_state: np.ndarray):
        """Solve one MPC step.

        Parameters
        ----------
        current_state : (10,) array [x, y, z, phi, psi, dx, dy, dz, dphi, dpsi]

        Returns
        -------
        u_opt : (5,) array — first control action [u_x, u_y, u_z, u_phi, u_psi]
        x_pred : (N+1, 10) array — predicted state trajectory
        """
        opti = ca.Opti()
        N = self.N
        dt = self.dt

        # ── Decision variables ───────────────────────────────────────────
        X = opti.variable(10, N + 1)  # states
        U = opti.variable(5, N)       # controls

        # ── Parameters ───────────────────────────────────────────────────
        x0 = opti.parameter(10)
        opti.set_value(x0, current_state)

        # Full 10D target: [x, y, z, phi, psi, dx, dy, dz, dphi, dpsi]
        # target is (4,) = [x, y, z, psi]; roll target is always 0
        x_target = np.zeros(10)
        x_target[0:3] = self.target[0:3]   # x, y, z
        x_target[3] = 0.0                   # phi = 0 (roll target)
        x_target[4] = self.target[3]         # psi (yaw target)
        # velocity targets = 0 (stop at target)

        # ── Dynamics (10D double integrator) ─────────────────────────────
        I5 = np.eye(5)
        Z5 = np.zeros((5, 5))
        A = np.block([[I5, I5 * dt], [Z5, I5]])
        B = np.block([[0.5 * I5 * dt**2], [I5 * dt]])

        # Initial condition
        opti.subject_to(X[:, 0] == x0)

        # ── Cost function ────────────────────────────────────────────────
        cost = 0
        Q_ca = ca.DM(self.Q)
        R_ca = ca.DM(self.R)

        for k in range(N):
            # State tracking cost
            dx = X[:, k] - x_target
            cost += ca.mtimes([dx.T, Q_ca, dx])

            # Control effort cost
            cost += ca.mtimes([U[:, k].T, R_ca, U[:, k]])

            # Velocity regularisation (penalise high speeds)
            v = X[5:10, k]
            cost += self.lam_vel * ca.dot(v, v)

            # Dynamics constraint
            opti.subject_to(X[:, k + 1] == ca.mtimes(ca.DM(A), X[:, k])
                            + ca.mtimes(ca.DM(B), U[:, k]))

        # Terminal cost (higher weight)
        dx_N = X[:, N] - x_target
        cost += 5.0 * ca.mtimes([dx_N.T, Q_ca, dx_N])

        opti.minimize(cost)

        # ── Constraints ──────────────────────────────────────────────────

        # Problem type constraints
        if self.problem_type == "descent_ascent":
            for k in range(N):
                opti.subject_to(U[0, k] == 0)  # no surge
                opti.subject_to(U[1, k] == 0)  # no sway
                opti.subject_to(U[4, k] == 0)  # no yaw
        elif self.problem_type == "lateral":
            for k in range(N):
                opti.subject_to(U[2, k] == 0)  # no heave

        for k in range(N):
            # Actuation limits
            for j in range(5):
                opti.subject_to(opti.bounded(-self.u_max, U[j, k], self.u_max))

            # Velocity limits
            for j in range(5):
                opti.subject_to(opti.bounded(-self.v_max, X[5 + j, k + 1], self.v_max))

            # Workspace bounds (position)
            opti.subject_to(opti.bounded(WS_X_MIN, X[0, k + 1], WS_X_MAX))
            opti.subject_to(opti.bounded(WS_Y_MIN, X[1, k + 1], WS_Y_MAX))
            opti.subject_to(opti.bounded(WS_Z_MIN, X[2, k + 1], WS_Z_MAX))

            # ── CBF obstacle constraints ─────────────────────────────────
            for obs in self.obstacles:
                p_obs = np.array(obs["position"])
                r_safe = float(obs["radius"]) + COLLISION_RADIUS

                # h(p) = ||p - p_obs||^2 - r_safe^2
                p_k = X[:3, k]        # position at step k
                p_k1 = X[:3, k + 1]   # position at step k+1

                h_k = ca.dot(p_k - p_obs, p_k - p_obs) - r_safe**2
                h_k1 = ca.dot(p_k1 - p_obs, p_k1 - p_obs) - r_safe**2

                # CBF constraint: h(k+1) >= (1 - gamma) * h(k)
                opti.subject_to(h_k1 >= (1 - self.gamma) * h_k)

        # ── Solver options ───────────────────────────────────────────────
        p_opts = {"expand": True, "print_time": False}
        s_opts = {
            "max_iter": 300,
            "print_level": 0,
            "sb": "yes",
            "warm_start_init_point": "yes",
        }
        opti.solver("ipopt", p_opts, s_opts)

        # ── Warm start ───────────────────────────────────────────────────
        if self._prev_x is not None:
            opti.set_initial(X, self._prev_x)
        if self._prev_u is not None:
            opti.set_initial(U, self._prev_u)

        # ── Solve ────────────────────────────────────────────────────────
        try:
            sol = opti.solve()
            x_opt = sol.value(X)
            u_opt = sol.value(U)

            # Store for warm start
            self._prev_x = x_opt
            self._prev_u = u_opt

            return np.array(u_opt[:, 0]).flatten(), np.array(x_opt.T)

        except RuntimeError as e:
            print(f"[MPC] Solver failed: {e}")
            # Return zero control on failure
            return np.zeros(5), None
