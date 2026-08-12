#!/usr/bin/env python3
"""Convert legacy SDK-end hand-eye samples to URDF base_link->tool0 poses."""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

from x5a_handeye.transforms import T_to_pose
from x5a_handeye.x5a_fk import fk_base_tool0


def pose_dict(joints):
    xyz, quat = T_to_pose(fk_base_tool0(joints))
    return {"xyz": xyz, "quat_xyzw": quat}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    source = Path(args.input)
    destination = Path(args.output)
    data = json.loads(source.read_text())
    converted = copy.deepcopy(data)
    samples = converted.get("samples", [])
    if not samples:
        raise SystemExit("no samples to convert")

    for sample in samples:
        joints = sample.get("joints")
        if joints is None or len(joints) < 6:
            raise SystemExit(f"sample {sample.get('index')} has no six-joint feedback")
        sample["T_sdk_base_end_raw"] = sample["T_base_tool"]
        sample["T_base_tool"] = pose_dict(joints)
        sample.setdefault("source", {})["robot"] = (
            "/arm_status.joint_pos -> URDF FK base_link->tool0"
        )

    converted["collection"] = "manual_gravity_readonly_urdf_fk"
    converted["base_frame"] = "base_link"
    converted["tool_frame"] = "tool0"
    converted["robot_pose_frame"] = "tool0"
    converted["converted_from"] = str(source)
    converted["control_commands_sent"] = False
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(converted, indent=2))
    print(f"converted {len(samples)} samples: {source} -> {destination}")


if __name__ == "__main__":
    main()
