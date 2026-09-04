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
    # Minimum time given to reach each new trajectory point, seconds.
    trajectory_point_duration_sec: float = 0.1
    # Largest single-joint distance (rad) still commanded at trajectory_point_duration_sec;
    # beyond this, the duration is scaled up proportionally. See FrankaRos2Follower's
    # docstring.
    joint_distance_threshold: float = 0.5
    # See FrankaRos2Follower's docstring: unverified for real hardware, adjust if a
    # per-joint contact test shows force-feedback pushes the wrong way.
    torque_sign: float = 1.0
    enable_gripper: bool = True
    # franka_gripper_node's control_msgs/action/GripperCommand action name.
    gripper_action_name: str = "/gripper_action"
    # Must match your leader config's gripper_teleop.actuation_range.
    gripper_actuation_range: float = 0.8
    # Franka Hand max opening width, meters. Assumes linear 0=closed mapping — verify.
    gripper_width_max: float = 0.08
    gripper_max_effort: float = 20.0
    gripper_goal_position_threshold: float = 0.005
    gripper_goal_refresh_period_sec: float = 0.1


def main(args: Args) -> None:
    if args.name == "left":
        zmq_addresses = franka_left_real_zmq_addresses
    elif args.name == "right":
        zmq_addresses = franka_right_real_zmq_addresses
    else:
        raise ValueError(f"Invalid name '{args.name}'. Expected 'left' or 'right'.")

    follower_main(
        zmq_addresses=zmq_addresses,
        name=args.name,
        node_name=f"factr_franka_ros2_follower_{args.name}",
        trajectory_topic=args.trajectory_topic,
        robot_state_topic=args.robot_state_topic,
        trajectory_point_duration_sec=args.trajectory_point_duration_sec,
        joint_distance_threshold=args.joint_distance_threshold,
        torque_sign=args.torque_sign,
        enable_gripper=args.enable_gripper,
        gripper_action_name=args.gripper_action_name,
        gripper_actuation_range=args.gripper_actuation_range,
        gripper_width_max=args.gripper_width_max,
        gripper_max_effort=args.gripper_max_effort,
        gripper_goal_position_threshold=args.gripper_goal_position_threshold,
        gripper_goal_refresh_period_sec=args.gripper_goal_refresh_period_sec,
    )


if __name__ == "__main__":
    main(tyro.cli(Args))
