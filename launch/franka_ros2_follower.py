import sys
from dataclasses import dataclass
from pathlib import Path

import tyro

# Case-safe path setup: import as lowercase "follower_robots"/"python_utils" rather than
# the "FACTR.*"-prefixed style used elsewhere, since that relies on Windows' case-insensitive
# filesystem matching "FACTR" to the actual "src/factr" directory — fragile on the Linux
# devcontainer. Verify separately if you ever hit an ImportError on the "FACTR.*" imports.
_SRC_FACTR = Path(__file__).parent.parent / "src" / "factr"
sys.path.insert(0, str(_SRC_FACTR))
sys.path.insert(0, str(_SRC_FACTR / "python_utils"))

from follower_robots.franka_ros2_follower import main as follower_main
from python_utils.global_configs import (
    franka_left_real_zmq_addresses,
    franka_right_real_zmq_addresses,
)


@dataclass
class Args:
    # Which side's ZMQ addresses to bind/connect to, must match the leader config's `name`.
    name: str = "right"
    # franka_ros2 topic this robot's joint_trajectory_controller listens on.
    trajectory_topic: str = "/joint_trajectory_controller/joint_trajectory"
    # franka_ros2 topic franka_robot_state_broadcaster publishes FrankaRobotState on.
    # Verify with `ros2 topic list` once the broadcaster is running — this is a best
    # guess, not confirmed.
    robot_state_topic: str = "/franka_robot_state_broadcaster/robot_state"
    # See FrankaRos2Follower's docstring: unverified for real hardware, adjust if a
    # per-joint contact test shows force-feedback pushes the wrong way.
    torque_sign: float = 1.0


def main(args: Args) -> None:
    if args.name == "left":
        zmq_addresses = franka_left_real_zmq_addresses
    elif args.name == "right":
        zmq_addresses = franka_right_real_zmq_addresses
    else:
        raise ValueError(f"Invalid name '{args.name}'. Expected 'left' or 'right'.")

    follower_main(
        zmq_addresses=zmq_addresses,
        node_name=f"factr_franka_ros2_follower_{args.name}",
        trajectory_topic=args.trajectory_topic,
        robot_state_topic=args.robot_state_topic,
        torque_sign=args.torque_sign,
    )


if __name__ == "__main__":
    main(tyro.cli(Args))
