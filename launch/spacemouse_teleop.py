"""Drives a FACTR follower (real franka_ros2 bridge or robosuite) with a 3Dconnexion
SpaceMouse instead of the Dynamixel exoskeleton -- see CartesianLeader's docstring for
why this solves IK leader-side and reuses the existing joint_pos_cmd_pub ZMQ channel
unmodified.

Requires `pyspacemouse` (`pip install pyspacemouse`) and a udev rule / permissions to
read the raw HID device on Linux (see the package's troubleshooting.md if `open()`
returns None -- typically a udev rule granting your user group access to the device).

Usage:
    python3 launch/spacemouse_teleop.py --side left
    python3 launch/spacemouse_teleop.py --side sim_right

NOT verified against real SpaceMouse hardware in this repo -- the field names read in
_SpaceMouseCartesianLeader.read_device() (state.x/.y/.z/.roll/.pitch/.yaw/.buttons)
match the long-standing pyspacemouse/SpaceNavigator state shape, but double check
against whatever version actually installs (e.g. run examples/pyspacemouse_test.py from
the package first) before trusting the mapping.
"""
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np
import tyro

_SRC_FACTR = Path(__file__).parent.parent / "src" / "factr"
sys.path.insert(0, str(_SRC_FACTR))
sys.path.insert(0, str(_SRC_FACTR / "python_utils"))
# factr_teleop is a nested ament_python ROS2 package too (src/factr/factr_teleop/
# factr_teleop/, same layout as python_utils above) -- needs the same extra insert,
# otherwise "factr_teleop" resolves to the outer (non-package) directory and importing
# factr_teleop.cartesian_leader fails.
sys.path.insert(0, str(_SRC_FACTR / "factr_teleop"))

from python_utils.global_configs import (
    franka_left_real_zmq_addresses,
    franka_right_real_zmq_addresses,
    franka_sim_left_zmq_addresses,
    franka_sim_right_zmq_addresses,
)

from factr_teleop.cartesian_leader import CartesianLeader, spin

_ZMQ_ADDRESSES = {
    "left": franka_left_real_zmq_addresses,
    "right": franka_right_real_zmq_addresses,
    "sim_left": franka_sim_left_zmq_addresses,
    "sim_right": franka_sim_right_zmq_addresses,
}


class _SpaceMouseCartesianLeader(CartesianLeader):
    def __init__(self, deadzone: float, gripper_button_index: int, **kwargs):
        import pyspacemouse

        self._pyspacemouse = pyspacemouse
        self._deadzone = deadzone
        self._gripper_button_index = gripper_button_index
        self._prev_gripper_button = False

        # pyspacemouse's API is module-level, not an object handle: open() returns a
        # bool (device opened or not) and reading happens via the module-level
        # pyspacemouse.read(), not a method on open()'s return value. Calling
        # `.read()` on that bool is what silently killed all motion before -- rclpy
        # logs (but doesn't crash on) exceptions raised inside a timer callback, so
        # this failed on every single tick without ever stopping the node.
        opened = pyspacemouse.open()
        if not opened:
            raise RuntimeError(
                "pyspacemouse.open() failed -- SpaceMouse not found or no "
                "permission to read the HID device (see the package's "
                "troubleshooting.md for the udev rule needed on Linux)."
            )
        super().__init__(**kwargs)

    def read_device(self) -> Tuple[np.ndarray, np.ndarray, bool, bool]:
        state = self._pyspacemouse.read()
        axes = np.array(
            [state.x, state.y, state.z, state.roll, state.pitch, state.yaw]
        )
        axes[np.abs(axes) < self._deadzone] = 0.0
        # SpaceMouse axes are a deflection (velocity-like), not an incremental delta
        # like robosuite's own driver -- scale by dt so it integrates as a velocity.
        dpos = axes[:3] * self._dt
        drot = axes[3:] * self._dt

        buttons = list(state.buttons) if state.buttons else []
        gripper_button = (
            bool(buttons[self._gripper_button_index])
            if len(buttons) > self._gripper_button_index
            else False
        )
        gripper_toggle = gripper_button and not self._prev_gripper_button
        self._prev_gripper_button = gripper_button

        should_stop = False
        return dpos, drot, gripper_toggle, should_stop


@dataclass
class Args:
    side: str = "left"  # left, right, sim_left, sim_right
    control_freq: float = 100.0
    translation_scale: float = 0.3
    rotation_scale: float = 0.5
    deadzone: float = 0.05
    gripper_button_index: int = 0
    gripper_actuation_range: float = 0.08


def main(args: Args) -> None:
    if args.side not in _ZMQ_ADDRESSES:
        raise ValueError(f"Invalid side '{args.side}'. Expected one of {list(_ZMQ_ADDRESSES)}.")

    leader = _SpaceMouseCartesianLeader(
        deadzone=args.deadzone,
        gripper_button_index=args.gripper_button_index,
        name=args.side,
        zmq_addresses=_ZMQ_ADDRESSES[args.side],
        control_freq=args.control_freq,
        translation_scale=args.translation_scale,
        rotation_scale=args.rotation_scale,
        gripper_actuation_range=args.gripper_actuation_range,
        node_name=f"spacemouse_leader_{args.side}",
    )
    spin(leader)


if __name__ == "__main__":
    import rclpy

    rclpy.init()
    main(tyro.cli(Args))
