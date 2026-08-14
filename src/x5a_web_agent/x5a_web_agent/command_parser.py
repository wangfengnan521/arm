#!/usr/bin/env python3
"""Natural-language command parser for X5A voice pick-place.

First version is rule/keyword based. Later swap in LLMCommandParser.
The parser may only emit the whitelist below. It never emits joint
angles, Cartesian trajectories, /arm_cmd, shell, or Python code.
"""
from __future__ import annotations

import json
import re
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional


ALLOWED_ACTIONS = frozenset({"pick_place", "unknown", "cancel", "sequence"})
ALLOWED_COLORS = frozenset({"red", "white", "orange"})
ALLOWED_SELECTORS = frozenset({"nearest", "farthest"})
ALLOWED_TARGETS = ALLOWED_COLORS | ALLOWED_SELECTORS

COLOR_ALIASES = {
    "red": ("red", "红", "红色", "红的", "红方块", "红色方块", "红色的"),
    "white": ("white", "白", "白色", "白的", "白方块", "白色方块", "白色的"),
    "orange": (
        "orange",
        "橙",
        "橙色",
        "橘色",
        "桔色",
        "橙的",
        "橘的",
        "橙色方块",
        "橘色方块",
        "橙色的",
        "橘色的",
    ),
}

PICK_HINTS = (
    "抓",
    "拿",
    "取",
    "夹",
    "捡",
    "挑",
    "搬走",
    "拿走",
    "抓取",
    "帮我",
    "请",
    "pick",
    "grasp",
    "grab",
)

STOP_HINTS = ("停止", "停下", "取消", "别抓", "不要动", "急停", "stop", "cancel")

SEQUENCE_HINTS = (
    "先", "再", "然后", "接着", "之后", "和", "跟", "以及", "还有", "与",
    "都", "全部", "两个", "三个", "and then", "then", " and ",
)
NEAREST_HINTS = ("最近", "最近的", "离得最近", "离盒子最近", "nearest", "closest")
FARTHEST_HINTS = ("最远", "最远的", "离得最远", "离盒子最远", "farthest", "furthest")

UNKNOWN_RESULT = {
    "action": "unknown",
    "message": "暂时无法理解该指令",
}


def _normalize(text: str) -> str:
    return re.sub(r"\s+", "", (text or "").strip().lower())


def _find_colors_in_order(text: str) -> List[str]:
    """Return colors in the order they appear in the utterance."""
    hits: List[tuple[int, str]] = []
    for color, aliases in COLOR_ALIASES.items():
        for alias in sorted(aliases, key=len, reverse=True):
            idx = text.find(alias.lower() if alias.isascii() else alias)
            # Chinese aliases are matched against the original mixed string.
            idx = text.find(alias) if idx < 0 else idx
            if idx < 0:
                idx = text.find(alias.lower())
            if idx >= 0:
                hits.append((idx, color))
                break
    hits.sort(key=lambda item: item[0])
    seen = set()
    ordered: List[str] = []
    for _, color in hits:
        if color not in seen:
            seen.add(color)
            ordered.append(color)
    return ordered


def sanitize_task(task: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Reject anything outside the first-version whitelist."""
    if not isinstance(task, dict):
        return None
    action = str(task.get("action", "")).strip()
    if action not in ALLOWED_ACTIONS:
        return None
    if action == "pick_place":
        color = str(task.get("target_color", "")).strip().lower()
        if color not in ALLOWED_TARGETS:
            return None
        return {"action": "pick_place", "target_color": color}
    if action == "cancel":
        return {"action": "cancel"}
    if action == "sequence":
        tasks = []
        for item in task.get("tasks") or []:
            cleaned = sanitize_task(item)
            if cleaned is None or cleaned.get("action") != "pick_place":
                return None
            tasks.append(cleaned)
        if not tasks:
            return None
        return {"action": "sequence", "tasks": tasks}
    return {
        "action": "unknown",
        "message": str(task.get("message") or UNKNOWN_RESULT["message"]),
    }


class CommandParser(ABC):
    @abstractmethod
    def parse_command(self, text: str) -> Dict[str, Any]:
        raise NotImplementedError


class RuleBasedCommandParser(CommandParser):
    def parse_command(self, text: str) -> Dict[str, Any]:
        raw = (text or "").strip()
        if not raw:
            return dict(UNKNOWN_RESULT)
        compact = _normalize(raw)
        if any(hint in compact for hint in STOP_HINTS):
            return {"action": "cancel"}

        colors = _find_colors_in_order(compact)
        looks_like_sequence = (
            len(colors) >= 2 and any(hint in compact for hint in SEQUENCE_HINTS)
        )
        if looks_like_sequence:
            return sanitize_task(
                {
                    "action": "sequence",
                    "tasks": [
                        {"action": "pick_place", "target_color": color}
                        for color in colors
                    ],
                }
            ) or dict(UNKNOWN_RESULT)

        has_nearest = any(hint in compact for hint in NEAREST_HINTS)
        has_farthest = any(hint in compact for hint in FARTHEST_HINTS)
        if has_farthest and not colors:
            return {"action": "pick_place", "target_color": "farthest"}
        if has_nearest and not colors:
            return {"action": "pick_place", "target_color": "nearest"}

        if not colors:
            return dict(UNKNOWN_RESULT)

        # A lone color, or any utterance that names one allowed color, is pick_place.
        return {"action": "pick_place", "target_color": colors[0]}


class LLMCommandParser(CommandParser):
    """Reserved hook. An LLM may only fill this JSON schema.

    Allowed output:
      {"action":"pick_place","target_color":"red"|"white"|"orange"}
      {"action":"unknown","message":"..."}
      {"action":"cancel"}
      {"action":"sequence","tasks":[{"action":"pick_place","target_color":"..."}]}

    Forbidden output:
      joint angles, Cartesian trajectories, /arm_cmd, shell, ros2 topic pub,
      Python code, or any extra keys that would bypass MoveIt.
    """

    SCHEMA = {
        "type": "object",
        "required": ["action"],
        "additionalProperties": False,
        "properties": {
            "action": {
                "type": "string",
                "enum": sorted(ALLOWED_ACTIONS),
            },
            "target_color": {
                "type": "string",
                "enum": sorted(ALLOWED_TARGETS),
            },
            "message": {"type": "string"},
            "tasks": {
                "type": "array",
                "items": {
                    "type": "object",
                    "required": ["action", "target_color"],
                    "additionalProperties": False,
                    "properties": {
                        "action": {"type": "string", "enum": ["pick_place"]},
                        "target_color": {
                            "type": "string",
                            "enum": sorted(ALLOWED_TARGETS),
                        },
                    },
                },
            },
        },
    }

    def __init__(self, fallback: Optional[CommandParser] = None) -> None:
        self.fallback = fallback or RuleBasedCommandParser()

    def parse_command(self, text: str) -> Dict[str, Any]:
        # First version does not call a model. Keep the rule parser in charge.
        return self.fallback.parse_command(text)

    @staticmethod
    def validate_model_json(payload: Any) -> Dict[str, Any]:
        if isinstance(payload, str):
            try:
                payload = json.loads(payload)
            except json.JSONDecodeError:
                return dict(UNKNOWN_RESULT)
        cleaned = sanitize_task(payload) if isinstance(payload, dict) else None
        return cleaned or dict(UNKNOWN_RESULT)


def parse_command(text: str) -> Dict[str, Any]:
    return RuleBasedCommandParser().parse_command(text)


def _self_test() -> None:
    parser = RuleBasedCommandParser()
    cases = {
        "抓红色": {"action": "pick_place", "target_color": "red"},
        "把红色方块拿走": {"action": "pick_place", "target_color": "red"},
        "帮我拿一下红色的": {"action": "pick_place", "target_color": "red"},
        "红色": {"action": "pick_place", "target_color": "red"},
        "抓一下那个红色方块": {"action": "pick_place", "target_color": "red"},
        "白色": {"action": "pick_place", "target_color": "white"},
        "橙色": {"action": "pick_place", "target_color": "orange"},
        "橘色": {"action": "pick_place", "target_color": "orange"},
        "orange": {"action": "pick_place", "target_color": "orange"},
        "停止": {"action": "cancel"},
        "先抓红色，再抓白色": {
            "action": "sequence",
            "tasks": [
                {"action": "pick_place", "target_color": "red"},
                {"action": "pick_place", "target_color": "white"},
            ],
        },
        "请把红色和白色的方块放入盒子里": {
            "action": "sequence",
            "tasks": [
                {"action": "pick_place", "target_color": "red"},
                {"action": "pick_place", "target_color": "white"},
            ],
        },
        "把最近的方块放入盒子": {"action": "pick_place", "target_color": "nearest"},
        "把最远的放入盒子里": {"action": "pick_place", "target_color": "farthest"},
        "跳舞": UNKNOWN_RESULT,
        "": UNKNOWN_RESULT,
    }
    failed = 0
    for text, expected in cases.items():
        got = parser.parse_command(text)
        if got != expected:
            failed += 1
            print(f"FAIL {text!r}: {got} != {expected}")
    if failed:
        raise SystemExit(failed)
    print("command_parser self-test: PASS")


if __name__ == "__main__":
    _self_test()
