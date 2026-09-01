from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launches two franka.launch.py instances side by side, each pushed into its own
    ROS2 namespace ('left'/'right'), so their /controller_manager, joint_states, and
    other node-relative topics/services don't collide. franka_ros2 v0.1.0 has no
    built-in multi-robot namespace support, so this wraps its launch file from the
    outside instead of editing the (git-cloned, third-party) franka_ros2 source.

    After this is running, spawn joint_trajectory_controller per side explicitly
    (not done here, since franka.launch.py doesn't spawn it):
        ros2 run controller_manager spawner joint_trajectory_controller \\
            --controller-manager /left/controller_manager
        ros2 run controller_manager spawner joint_trajectory_controller \\
            --controller-manager /right/controller_manager

    Then run franka_ros2_follower.py per side with matching namespaced topics:
        python launch/franka_ros2_follower.py --name left \\
            --trajectory-topic /left/joint_trajectory_controller/joint_trajectory \\
            --robot-state-topic /left/franka_robot_state_broadcaster/robot_state
        python launch/franka_ros2_follower.py --name right \\
            --trajectory-topic /right/joint_trajectory_controller/joint_trajectory \\
            --robot-state-topic /right/franka_robot_state_broadcaster/robot_state

    Known side effect: /tf and /tf_static also end up namespaced (/left/tf, /right/tf)
    rather than merged into one shared tree. Irrelevant to FACTR itself (the follower
    bridge reads FrankaRobotState.q directly, not TF), but matters if you later want a
    single combined RViz view of both arms.
    """
    left_robot_ip = DeclareLaunchArgument(
        "left_robot_ip", description="Hostname or IP address of the left robot"
    )
    right_robot_ip = DeclareLaunchArgument(
        "right_robot_ip", description="Hostname or IP address of the right robot"
    )
    use_fake_hardware = DeclareLaunchArgument("use_fake_hardware", default_value="false")
    load_gripper = DeclareLaunchArgument("load_gripper", default_value="true")

    franka_launch = PathJoinSubstitution(
        [FindPackageShare("franka_bringup"), "launch", "franka.launch.py"]
    )

    def side_group(namespace: str, robot_ip_arg: str):
        return GroupAction(
            [
                PushRosNamespace(namespace),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(franka_launch),
                    launch_arguments={
                        "robot_ip": LaunchConfiguration(robot_ip_arg),
                        "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                        "load_gripper": LaunchConfiguration("load_gripper"),
                    }.items(),
                ),
            ]
        )

    return LaunchDescription(
        [
            left_robot_ip,
            right_robot_ip,
            use_fake_hardware,
            load_gripper,
            side_group("left", "left_robot_ip"),
            side_group("right", "right_robot_ip"),
        ]
    )
