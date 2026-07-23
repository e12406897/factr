import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import tyro

# Add parent directory to path so we can import src module
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "src" / "python_utils"))
from python_utils.global_configs import franka_sim_zmq_addresses
from src.follower_robots.sim_franka_follower import MujocoFrankaFollower
MENAGERIE_ROOT: Path = (
    Path(__file__).parent.parent / "src" / "mujoco_menagerie"
)
xml = MENAGERIE_ROOT / "franka_fr3" / "fr3.xml"
gripper_xml = MENAGERIE_ROOT / "franka_emika_panda" / "hand.xml"
    


@dataclass
class Args:
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    robot_ip: str = "192.168.1.10"
    # sim_fr3_franka only: leader/follower name, must match the teleop config's `name` field in factr_teleop_franka_zmq.py
    follower_name: str = "sim"
    # sim_fr3_franka only: mirror the real Franka's ROS gripper command/feedback topics
    enable_ros_gripper: bool = False
    # sim_fr3_franka only: initial arm joint configuration (7 values, radians). Set this
    # to match your teleop config's `initial_match_joint_pos` so the sim and leader line
    # up visually before the first command arrives.
    initial_arm_qpos: Optional[Tuple[float, float, float, float, float, float, float]] = None
    # sim_fr3_franka only: initial gripper command (radians, same convention as
    # `leader_gripper_pos`). Set this to your teleop config's `gripper_teleop.actuation_range`
    # (fully open) so the fingers don't self-contact before the leader sends a command.
    initial_gripper_cmd: float = 0.0


def launch_robot_server(args: Args):
    port = args.robot_port
    
    # Makes the sim speak FACTR's ZMQ PUB/SUB protocol directly, acting as a
    # drop-in replacement for the real Franka follower so that the unmodified
    # `factr_teleop_franka_zmq` leader node can teleoperate the sim.

    
    follower = MujocoFrankaFollower(
        xml_path=xml,
        gripper_xml_path=gripper_xml,
        zmq_addresses=franka_sim_zmq_addresses,
        enable_ros_gripper=args.enable_ros_gripper,
        name=args.follower_name,
        initial_arm_qpos=args.initial_arm_qpos,
        initial_gripper_cmd=args.initial_gripper_cmd,
    )
    follower.serve()
    
        


def main(args):
    launch_robot_server(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
