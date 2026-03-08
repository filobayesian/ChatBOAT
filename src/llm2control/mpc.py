import numpy as np
import cvxpy as cp

class MPCSolver:
    def __init__(self, dt=0.1, horizon=15):
        self.dt = dt
        self.T = horizon
        # Linear Double Integrator Dynamics (Simplified)
        self.A = np.array([[1, dt], [0, 1]])
        self.B = np.array([[0.5*dt**2], [dt]])

    def compute_step(self, current_state, target_val, q_p, r_c, v_max):
        """Solves one iteration of the MPC."""
        x = cp.Variable((2, self.T + 1))
        u = cp.Variable((1, self.T))
        
        cost = 0
        constraints = [x[:, 0] == current_state]
        
        for t in range(self.T):
            cost += q_p * cp.square(x[0, t] - target_val) + r_c * cp.square(u[0, t])
            constraints += [x[:, t+1] == self.A @ x[:, t] + self.B @ u[:, t]]
            constraints += [cp.abs(x[1, t]) <= v_max] # Velocity constraint

        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP, warm_start=True)
        return u[0, 0].value if prob.status == cp.OPTIMAL else 0.0