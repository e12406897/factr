"""Drives a FACTR follower (real franka_ros2 bridge or robosuite) with a 3Dconnexion
SpaceMouse, using robosuite 1.5.2's own SpaceMouse driver and device->action processing
(robosuite/devices/spacemouse.py + Device.input2action) feeding a copy of robosuite's
IK_POSE controller (see CartesianLeader).

Controls (robosuite's): move/twist the cap to move/rotate the end effector, hold the
left button to close the gripper, right button re-zeroes the device.

Requires the `hidapi` pip package (provides `import hid` with `hid.device()`, which
robosuite's driver uses) -- NOT the ctypes `hid` package; uninstall that one first if
present, both install a module called `hid`.

Usage:
    python3 launch/spacemouse_teleop.py --side left
    python3 launch/spacemouse_teleop.py --side sim_right
"""
import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Tuple

import numpy as np
import tyro

_SRC_FACTR = Path(__file__).parent.parent / "src" / "factr"
sys.path.insert(0, str(_SRC_FACTR))
sys.path.insert(0, str(_SRC_FACTR / "python_utils"))
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
    def __init__(self, pos_sensitivity: float, rot_sensitivity: float, **kwargs):
        # Imported from the submodule directly: robosuite.devices swallows the ImportError
        # (e.g. missing `hid`) and just prints a warning.
        from robosuite.devices.spacemouse import SpaceMouse

        # robosuite's Device base only reads env.robots[i].arms (for multi-arm switching).
        stub_env = SimpleNamespace(robots=[SimpleNamespace(arms=["right"])])
        self._device = SpaceMouse(
            env=stub_env,
            pos_sensitivity=pos_sensitivity,
            rot_sensitivity=rot_sensitivity,
        )
        self._device.start_control()
        super().__init__(**kwargs)

    def read_device(self) -> Tuple[np.ndarray, np.ndarray, bool, bool]:
        state = self._device.get_controller_state()
        if state["reset"]:
            self._device.start_control()
            return np.zeros(3), np.zeros(3), False, False

        # --- robosuite Device.input2action (mirror_actions=False) ---
        dpos = state["dpos"]
        raw_drotation = state["raw_drotation"]
        drotation = raw_drotation[[1, 0, 2]]
        drotation[2] = -drotation[2]
        dpos, drotation = self._device._postprocess_device_outputs(dpos, drotation)
        dpos = np.clip(dpos, -1, 1)
        drotation = np.clip(drotation, -1, 1)

        return dpos, drotation, bool(state["grasp"]), False


@dataclass
class Args:
    side: str = "left"  # left, right, sim_left, sim_right
    control_freq: float = 20.0
    pos_sensitivity: float = 1.0
    rot_sensitivity: float = 1.0


def main(args: Args) -> None:
    if args.side not in _ZMQ_ADDRESSES:
        raise ValueError(f"Invalid side '{args.side}'. Expected one of {list(_ZMQ_ADDRESSES)}.")

    leader = _SpaceMouseCartesianLeader(
        pos_sensitivity=args.pos_sensitivity,
        rot_sensitivity=args.rot_sensitivity,
        name=args.side,
        zmq_addresses=_ZMQ_ADDRESSES[args.side],
        control_freq=args.control_freq,
        node_name=f"spacemouse_leader_{args.side}",
    )
    spin(leader)


if __name__ == "__main__":
    import rclpy

    rclpy.init()
    main(tyro.cli(Args))
