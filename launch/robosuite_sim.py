"""Robosuite-based follower sim -- the robosuite counterpart to mujoco_sim.py, with the
same ZMQ/torque-feedback behaviour but robosuite's ready-made task environments
(objects, reward functions) instead of a bare MuJoCo scene.

Requires `robosuite` (`pip install robosuite`).

Usage:
    # single arm, one process per side (same model as mujoco_sim.py)
    python3 launch/robosuite_sim.py --side left
    python3 launch/robosuite_sim.py --side right --env-name Stack

    # bimanual: ONE process driving both leaders through one shared TwoArm* env
    python3 launch/robosuite_sim.py --side both

Then start the unmodified FACTR leader(s) as usual:
    ros2 launch launch/factr_teleop.py side:=sim_left
    ros2 launch launch/factr_teleop.py side:=sim_right
"""
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Optional

import tyro
import yaml

_SRC_FACTR = Path(__file__).parent.parent / "src" / "factr"
sys.path.insert(0, str(_SRC_FACTR))
sys.path.insert(0, str(_SRC_FACTR / "python_utils"))

from python_utils.global_configs import (
    franka_sim_left_zmq_addresses,
    franka_sim_right_zmq_addresses,
)
from python_utils.utils import get_workspace_root

from follower_robots.robosuite_franka_follower import RobosuiteFrankaFollower

_SIDE_CONFIG = {
    "left": (franka_sim_left_zmq_addresses, "sim_left", "franka_sim_left.yaml"),
    "right": (franka_sim_right_zmq_addresses, "sim_right", "franka_sim_right.yaml"),
}


@dataclass
class Args:
    # Which leader/follower pair(s) this sim serves. "left"/"right" run a single-arm env
    # (one process per side, like mujoco_sim.py); "both" runs ONE process with a shared
    # TwoArm* env, which a real bimanual task requires (one physics step drives both).
    side: Literal["left", "right", "both"] = "left"
    # robosuite env id. Defaults to "Lift" for a single arm and "TwoArmLift" for "both".
    # Other two-arm options: TwoArmPegInHole, TwoArmHandover, TwoArmTransport.
    env_name: Optional[str] = None
    # TwoArm envs only (ignored for single-arm). If "default" errors out, robosuite's
    # exception message lists the values your installed version accepts.
    env_configuration: str = "default"
    # Table height in meters (Lift/TwoArmLift). None keeps robosuite's hardcoded 0.8;
    # increase it to raise the table relative to the robot base.
    table_offset_z: Optional[float] = None
    has_renderer: bool = True
    control_freq: int = 20
    # JOINT_POSITION controller impedance gains.
    kp: float = 150.0
    damping_ratio: float = 1.0
    enable_ros_gripper: bool = True
    # Same adaptive external-torque smoothing as mujoco_sim.py (defaults matched to it).
    enable_var_scale_feedback: bool = True
    var_scale_factor: float = 10.0


def _read_gripper_actuation_range(config_file: str) -> float:
    config_path = os.path.join(
        get_workspace_root(),
        f"src/factr/factr_teleop/factr_teleop/configs/{config_file}",
    )
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return float(config["gripper_teleop"]["actuation_range"])


def main(args: Args) -> None:
    sides = ["left", "right"] if args.side == "both" else [args.side]
    zmq_addresses, names, config_files = zip(*(_SIDE_CONFIG[s] for s in sides))

    env_name = args.env_name
    if env_name is None:
        env_name = "TwoArmLift" if args.side == "both" else "Lift"

    follower = RobosuiteFrankaFollower(
        zmq_addresses=list(zmq_addresses),
        gripper_actuation_range=[_read_gripper_actuation_range(c) for c in config_files],
        names=list(names),
        env_name=env_name,
        env_configuration=args.env_configuration,
        table_offset_z=args.table_offset_z,
        enable_ros_gripper=args.enable_ros_gripper,
        has_renderer=args.has_renderer,
        control_freq=args.control_freq,
        kp=args.kp,
        damping_ratio=args.damping_ratio,
        enable_var_scale_feedback=args.enable_var_scale_feedback,
        var_scale_factor=args.var_scale_factor,
    )
    follower.serve()


if __name__ == "__main__":
    main(tyro.cli(Args))
