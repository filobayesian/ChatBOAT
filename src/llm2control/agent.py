import json
from openai import OpenAI

class ClaudeAgent:
    def __init__(self, api_key):
        self.client = OpenAI(
            api_key=api_key,
            base_url="https://openrouter.ai/api/v1"
        )

    def get_maneuver_plan(self, user_prompt):
        """Calls Claude via OpenRouter to get MPC weights and timing."""
        response = self.client.chat.completions.create(
            model="anthropic/claude-sonnet-4.6",
            max_tokens=1024,
            tools=[{
                "type": "function",
                "function": {
                    "name": "translation_skill",
                    "description": "Set MPC weights and total simulation time for a maneuver.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "target": {"type": "array", "items": {"type": "number"}, "description": "[x, y, z]"},
                            "weights": {
                                "type": "object",
                                "properties": {
                                    "Q": {"type": "array", "items": {"type": "number"}, "description": "Position weight"},
                                    "R": {"type": "array", "items": {"type": "number"}, "description": "Control effort weight"}
                                }
                            },
                            "sim_time": {"type": "number", "description": "Total time in seconds to run this maneuver"}
                        },
                        "required": ["target", "weights", "sim_time"]
                    }
                }
            }],
            tool_choice={"type": "function", "function": {"name": "translation_skill"}},
            messages=[{"role": "user", "content": user_prompt}]
        )
        # Parse the tool call arguments from JSON string to dict
        return json.loads(response.choices[0].message.tool_calls[0].function.arguments)


if __name__ == "__main__":
    pass
