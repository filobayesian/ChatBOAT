import numpy as np

class ManeuverParser:
    @staticmethod
    def parse(json_data):
        """Extracts and validates MPC parameters."""
        try:
            config = {
                "target": np.array(json_data['target']),
                "Q": np.array(json_data['weights']['Q']),
                "R": np.array(json_data['weights']['R']),
                "t_sim": float(json_data['sim_time']),
                "v_max": float(json_data.get('v_max', 2.0)) # Default safety
            }
            return config
        except KeyError as e:
            print(f"Parser Error: Missing field {e}")
            return None