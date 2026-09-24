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


_3DCONNEXION_VENDOR_ID = 0x256F
_LOGITECH_VENDOR_ID = 0x046D  # older 3Dconnexion devices were sold under Logitech's ID


def _hid_backend():
    # hidapi's `hid` module uses the libusb backend on Linux; `hidraw` (same API, same
    # package) goes through /dev/hidraw* like pyspacemouse did.
    try:
        import hidraw

        return hidraw
    except ImportError:
        import hid

        return hid


def _find_spacemouse(hid_module, device_path: str = ""):
    devices = hid_module.enumerate()
    if device_path:
        matches = [d for d in devices if d["path"] == device_path.encode()]
    else:
        matches = [
            d
            for d in devices
            if d["vendor_id"] == _3DCONNEXION_VENDOR_ID
            or (
                d["vendor_id"] == _LOGITECH_VENDOR_ID
                and "3dconnexion"
                in f"{d.get('manufacturer_string')} {d.get('product_string')}".lower()
            )
        ]
    if not matches:
        listing = "\n".join(
            f"  {d['vendor_id']:04x}:{d['product_id']:04x} {d['path']} "
            f"{d.get('manufacturer_string')!r} {d.get('product_string')!r}"
            for d in devices
        )
        raise RuntimeError(
            "No 3Dconnexion device found via "
            f"{hid_module.__name__}. Visible HID devices:\n{listing or '  (none)'}"
        )
    return sorted(matches, key=lambda d: d.get("interface_number", 0))[0]


class _SpaceMouseCartesianLeader(CartesianLeader):
    def __init__(
        self, pos_sensitivity: float, rot_sensitivity: float, device_path: str, **kwargs
    ):
        # Imported from the submodule directly: robosuite.devices swallows the ImportError
        # (e.g. missing `hid`) and just prints a warning.
        import robosuite.devices.spacemouse as rs_spacemouse

        hid_module = _hid_backend()
        rs_spacemouse.hid = hid_module
        info = _find_spacemouse(hid_module, device_path)

        # robosuite's Device base only reads env.robots[i].arms (for multi-arm switching).
        stub_env = SimpleNamespace(robots=[SimpleNamespace(arms=["right"])])
        # Real product_id matters: robosuite parses the older model (0xc635) differently.
        self._device = rs_spacemouse.SpaceMouse(
            env=stub_env,
            vendor_id=info["vendor_id"],
            product_id=info["product_id"],
            device_path=info["path"],
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
    # e.g. /dev/hidraw3 -- only needed if the auto-detected device/interface is wrong
    device_path: str = ""


def main(args: Args) -> None:
    if args.side not in _ZMQ_ADDRESSES:
        raise ValueError(f"Invalid side '{args.side}'. Expected one of {list(_ZMQ_ADDRESSES)}.")

    leader = _SpaceMouseCartesianLeader(
        pos_sensitivity=args.pos_sensitivity,
        rot_sensitivity=args.rot_sensitivity,
        device_path=args.device_path,
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
