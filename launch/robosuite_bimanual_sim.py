"""Launches a shared robosuite TwoArm* bimanual sim (two Panda arms with the standard
PandaGripper) and bridges it to both FACTR ZMQ leaders (sim_left, sim_right) at once.

Requires `robosuite` (not installed by default -- `pip install robosuite`).

Usage:
    python3 launch/robosuite_bimanual_sim.py
    python3 launch/robosuite_bimanual_sim.py --env-name TwoArmPegInHole --no-has-renderer

Then, in two other terminals, start the unmodified FACTR leaders as usual:
    ros2 launch launch/factr_teleop.py side:=sim_left
    ros2 launch launch/factr_teleop.py side:=sim_right
"""
import os
import sys
from dataclasses import dataclass
from pathlib import Path

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

from follower_robots.robosuite_bimanual_follower import RobosuiteBimanualFollower


@dataclass
class Args:
    env_name: str = "TwoArmLift"
    # See TwoArmLift's docstring for valid values (e.g. "opposed", "parallel") -- if
    # "default" errors out, robosuite's exception message lists what your installed
    # version accepts for this env.
    env_configuration: str = "default"
    has_renderer: bool = True
    control_freq: int = 20
    kp: float = 150.0
    damping_ratio: float = 1.0
    enable_ros_gripper: bool = True
    # Leader config files (used only to read gripper_teleop.actuation_range, so the
    # leader->gripper-fraction mapping matches what the leaders actually send).
    config_file_left: str = "franka_sim_left.yaml"
    config_file_right: str = "franka_sim_right.yaml"


def _read_gripper_actuation_range(config_file: str) -> float:
    config_path = os.path.join(
        get_workspace_root(),
        f"src/factr/factr_teleop/factr_teleop/configs/{config_file}",
    )
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return float(config["gripper_teleop"]["actuation_range"])


def main(args: Args) -> None:
    follower = RobosuiteBimanualFollower(
        zmq_addresses_left=franka_sim_left_zmq_addresses,
        zmq_addresses_right=franka_sim_right_zmq_addresses,
        gripper_actuation_range_left=_read_gripper_actuation_range(args.config_file_left),
        gripper_actuation_range_right=_read_gripper_actuation_range(args.config_file_right),
        name_left="sim_left",
        name_right="sim_right",
        env_name=args.env_name,
        env_configuration=args.env_configuration,
        enable_ros_gripper=args.enable_ros_gripper,
        has_renderer=args.has_renderer,
        control_freq=args.control_freq,
        kp=args.kp,
        damping_ratio=args.damping_ratio,
    )
    follower.serve()


if __name__ == "__main__":
    main(tyro.cli(Args))
