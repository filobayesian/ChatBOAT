"""LLM agent with dual Task Planner + Optimization Formulator roles.

Uses Claude via OpenRouter (openai SDK) with tool_choice for structured output.
"""

import json
import logging
import math
import os
from datetime import datetime, timezone
from pathlib import Path

from openai import OpenAI

from llm2control.config import KNOWN_OBJECTS, ROBOT_START
from llm2control.prompts.task_planner import (
    TASK_PLANNER_SYSTEM,
    TASK_PLANNER_TOOL,
    TASK_PLANNER_EXAMPLES,
)
from llm2control.prompts.optimization_formulator import (
    OPTIMIZATION_FORMULATOR_SYSTEM,
    OPTIMIZATION_FORMULATOR_TOOL,
    OPTIMIZATION_FORMULATOR_EXAMPLES,
)
from llm2control.parser import Subtask, MPCConfig, parse_subtasks, parse_mpc_config

logger = logging.getLogger(__name__)

# ── Audit log directory ─────────────────────────────────────────────────────
_LOG_DIR = Path(os.environ.get("LLM_LOG_DIR", "logs"))


def _write_audit_entry(entry: dict) -> None:
    """Append a JSON entry to the audit log file."""
    _LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = _LOG_DIR / "llm_audit.jsonl"
    with open(log_path, "a") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def _objects_description(objects: list[dict]) -> str:
    """Format known objects for inclusion in prompts."""
    lines = []
    for obj in objects:
        pos = obj["position"]
        lines.append(f"- {obj['name']}: position ({pos[0]}, {pos[1]}, {pos[2]}), "
                      f"bounding radius {obj['radius']} m")
    return "\n".join(lines) if lines else "- None"


class LaMPCAgent:
    """Dual-role LLM agent: Task Planner + Optimization Formulator."""

    def __init__(self, api_key: str, model: str = "anthropic/claude-sonnet-4-6"):
        self.client = OpenAI(
            api_key=api_key,
            base_url="https://openrouter.ai/api/v1",
        )
        self.model = model

    # ── Task Planner ─────────────────────────────────────────────────────────

    def plan_task(self, user_query: str, vehicle_state=None) -> list[Subtask]:
        """Decompose user command into ordered subtask list.

        Parameters
        ----------
        user_query : natural-language navigation command
        vehicle_state : optional (10,) array; uses ROBOT_START if None

        Returns
        -------
        list[Subtask]
        """
        if vehicle_state is not None:
            x, y, z = vehicle_state[0], vehicle_state[1], vehicle_state[2]
            psi = vehicle_state[4]  # 10D: [x,y,z,phi,psi,...]
        else:
            x, y, z = ROBOT_START
            psi = 0.0

        system_msg = TASK_PLANNER_SYSTEM.format(
            objects_description=_objects_description(KNOWN_OBJECTS),
            x=x, y=y, z=z, yaw=math.degrees(psi),
        )

        messages = [{"role": "system", "content": system_msg}]
        messages.extend(TASK_PLANNER_EXAMPLES)
        messages.append({"role": "user", "content": user_query})

        response = self.client.chat.completions.create(
            model=self.model,
            max_tokens=1024,
            tools=[TASK_PLANNER_TOOL],
            tool_choice={"type": "function", "function": {"name": "plan_subtasks"}},
            messages=messages,
        )

        raw_args = response.choices[0].message.tool_calls[0].function.arguments
        raw = json.loads(raw_args)

        _write_audit_entry({
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "role": "task_planner",
            "model": self.model,
            "input": {"user_query": user_query, "vehicle_state": [x, y, z, psi]},
            "output": raw,
            "usage": {
                "prompt_tokens": getattr(response.usage, "prompt_tokens", None),
                "completion_tokens": getattr(response.usage, "completion_tokens", None),
            },
        })
        logger.info("Task planner response logged to %s", _LOG_DIR / "llm_audit.jsonl")

        return parse_subtasks(raw)

    # ── Optimization Formulator ──────────────────────────────────────────────

    def formulate_optimization(self, subtask: Subtask,
                                vehicle_state=None) -> MPCConfig:
        """Produce MPC configuration for a single subtask.

        Parameters
        ----------
        subtask : the subtask to formulate
        vehicle_state : optional (10,) array; uses ROBOT_START if None

        Returns
        -------
        MPCConfig
        """
        if vehicle_state is not None:
            x, y, z = vehicle_state[0], vehicle_state[1], vehicle_state[2]
            psi = vehicle_state[4]  # 10D: [x,y,z,phi,psi,...]
        else:
            x, y, z = ROBOT_START
            psi = 0.0

        system_msg = OPTIMIZATION_FORMULATOR_SYSTEM.format(
            objects_description=_objects_description(KNOWN_OBJECTS),
            x=x, y=y, z=z, yaw=math.degrees(psi),
        )

        messages = [{"role": "system", "content": system_msg}]
        messages.extend(OPTIMIZATION_FORMULATOR_EXAMPLES)
        messages.append({
            "role": "user",
            "content": f"Subtask: {subtask.instruction}",
        })

        response = self.client.chat.completions.create(
            model=self.model,
            max_tokens=1024,
            tools=[OPTIMIZATION_FORMULATOR_TOOL],
            tool_choice={"type": "function", "function": {"name": "configure_mpc"}},
            messages=messages,
        )

        raw_args = response.choices[0].message.tool_calls[0].function.arguments
        raw = json.loads(raw_args)

        _write_audit_entry({
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "role": "optimization_formulator",
            "model": self.model,
            "input": {"subtask": subtask.instruction, "vehicle_state": [x, y, z, psi]},
            "output": raw,
            "usage": {
                "prompt_tokens": getattr(response.usage, "prompt_tokens", None),
                "completion_tokens": getattr(response.usage, "completion_tokens", None),
            },
        })
        logger.info("Optimization formulator response logged to %s", _LOG_DIR / "llm_audit.jsonl")

        # Handle unsupported tasks flagged by the optimization formulator
        if raw.get("problem_type") == "unsupported":
            raise ValueError(
                f"Optimization Formulator marked subtask as unsupported: "
                f"{subtask.instruction}"
            )

        return parse_mpc_config(raw)
