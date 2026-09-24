"""Drives a FACTR follower (real franka_ros2 bridge or robosuite) with a Force
Dimension Omega.6 instead of the Dynamixel exoskeleton -- see CartesianLeader's
docstring for why this solves IK leader-side and reuses the existing joint_pos_cmd_pub
ZMQ channel unmodified.

Prerequisite (NOT done yet as of writing this): the Force Dimension SDK
(libdhd/libdrd) must be installed and working standalone on this machine -- it is
proprietary, needs Force Dimension's own installer/drivers, and is not just a pip
install. Verify with one of their own SDK example binaries BEFORE trying this script;
if the SDK itself can't see the device, nothing here will either.

    pip install forcedimension-core

UNVERIFIED: the exact forcedimension_core.dhd function names/signatures used in
_Omega6CartesianLeader.read_device() (dhd.open, dhd.getPosition, dhd.getOrientationFrame,
dhd.getButton, dhd.close) are Force Dimension's long-standing, documented libdhd API
surface, but were not runnable/testable here (no Omega.6 hardware or SDK install
available in this environment). Check them against forcedimension_core's actual
installed docs/examples (`python3 -m pydoc forcedimension_core.dhd`) before trusting
this end to end -- this is the one part of the two new leader scripts that has not been
validated against a real device.

Usage:
    python3 launch/omega6_teleop.py --side left
    python3 launch/omega6_teleop.py --side sim_right
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


class _Omega6CartesianLeader(CartesianLeader):
    def __init__(self, gripper_button_index: int, **kwargs):
        import forcedimension_core.dhd as dhd

        self._dhd = dhd
        self._gripper_button_index = gripper_button_index
        self._prev_gripper_button = False
        self._prev_pos = None
        self._prev_rot = None

        device_id = dhd.open()
        if device_id < 0:
            raise RuntimeError(
                "dhd.open() failed to find an Omega.6 -- check the device is "
                "connected and that the Force Dimension SDK can see it on its own "
                "(run one of the SDK's own examples first)."
            )
        self._device_id = device_id
        super().__init__(**kwargs)

    def read_device(self) -> Tuple[np.ndarray, np.ndarray, bool, bool]:
        dhd = self._dhd
        # UNVERIFIED against real hardware -- see module docstring. Omega.6 reports
        # absolute position/orientation (not a per-tick delta like robosuite's own
        # SpaceMouse driver), so we diff against the previous reading ourselves.
        pos = np.array(dhd.getPosition())
        rot = np.array(dhd.getOrientationFrame())  # 3x3 rotation matrix

        if self._prev_pos is None:
            dpos = np.zeros(3)
            drot = np.zeros(3)
        else:
            dpos = pos - self._prev_pos
            # small-rotation axis-angle approx of the incremental rotation
            import pinocchio as pin

            drot = pin.log3(self._prev_rot.T @ rot)
        self._prev_pos, self._prev_rot = pos, rot

        gripper_button = bool(dhd.getButton(self._gripper_button_index))
        gripper_toggle = gripper_button and not self._prev_gripper_button
        self._prev_gripper_button = gripper_button

        should_stop = bool(dhd.getButton(0)) if self._gripper_button_index != 0 else False
        return dpos, drot, gripper_toggle, should_stop

    def destroy_node(self):
        self._dhd.close(self._device_id)
        super().destroy_node()


@dataclass
class Args:
    side: str = "left"  # left, right, sim_left, sim_right
    control_freq: float = 500.0  # Omega.6 supports much higher rates than a SpaceMouse
    translation_scale: float = 1.0
    rotation_scale: float = 1.0
    gripper_button_index: int = 0
    gripper_actuation_range: float = 0.08


def main(args: Args) -> None:
    if args.side not in _ZMQ_ADDRESSES:
        raise ValueError(f"Invalid side '{args.side}'. Expected one of {list(_ZMQ_ADDRESSES)}.")

    leader = _Omega6CartesianLeader(
        gripper_button_index=args.gripper_button_index,
        name=args.side,
        zmq_addresses=_ZMQ_ADDRESSES[args.side],
        control_freq=args.control_freq,
        translation_scale=args.translation_scale,
        rotation_scale=args.rotation_scale,
        gripper_actuation_range=args.gripper_actuation_range,
        node_name=f"omega6_leader_{args.side}",
    )
    spin(leader)


if __name__ == "__main__":
    import rclpy

    rclpy.init()
    main(tyro.cli(Args))
