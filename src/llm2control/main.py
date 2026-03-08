from llm2control import mpc, agent, parser
import numpy as np
import os

if __name__ == "__main__":
    # 1. Initialize Components
    control_agent = agent.ClaudeAgent(api_key=os.environ["OPENROUTER_API_KEY"])
    solver = mpc.MPCSolver(dt=0.1)

    # 2. Get Strategy from Claude
    raw_json = control_agent.get_maneuver_plan("Move to position x=10 meters quickly but smoothly.")
    plan = parser.ManeuverParser.parse(raw_json)

    if plan:
        # 3. Run the Control Loop for the specified Simulation Time
        current_state = np.array([0.0, 0.0]) # [pos, vel]
        num_steps = int(plan['t_sim'] / solver.dt)
        
        print(f"Starting {plan['t_sim']}s maneuver to target {plan['target']}...")
        
        for step in range(num_steps):
            # Run MPC Solve
            accel = solver.compute_step(
                current_state, 
                plan['target'][0], # Simplified to 1D for this example
                plan['Q'][0],
                plan['R'][0],
                plan['v_max']
            )
            
            # Apply to Simulator (or mock state update)
            current_state = solver.A @ current_state + solver.B.flatten() * accel
            
            if step % 10 == 0:
                print(f"Time: {step*0.1:.1f}s | Pos: {current_state[0]:.2f}m")