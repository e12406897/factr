"""Drives a FACTR follower (real franka_ros2 bridge or robosuite) with a Force Dimension
Omega.6, through the same robosuite-IK leader as the SpaceMouse (see CartesianLeader).

The Omega is a position device, so the handle pose is mapped ABSOLUTELY (relative to
where handle and robot were at startup) and each tick feeds `target - current` into the
IK step -- the same error form robosuite's IK_POSE.run_controller uses. The robot then
tracks the handle and catches up if it lags, instead of losing deltas.

Prerequisites:
  - Force Dimension SDK >= 3.16 (proprietary, download from forcedimension.com) extracted
    to $FDSDK (set in the Dockerfile to /factr/third_party/forcedimension_sdk), so that
    $FDSDK/lib/release/lin-x86_64-gcc/libdrd.so.* exists.
  - `pip install forcedimension-core` (in requirements.txt).
  - USB access for the non-root container user: udev rule on the HOST, e.g.
        SUBSYSTEM=="usb", ATTRS{idVendor}=="1451", MODE="0666"
    (check the vendor ID with `lsusb`).

Startup: if the device isn't calibrated yet, it moves by itself (drd.autoInit) -- keep
hands off the handle until "ready" is logged. Afterwards the SDK's gravity compensation
holds the handle, and button 0 toggles the gripper.

Usage:
    python3 launch/omega6_teleop.py --side left
    python3 launch/omega6_teleop.py --side sim_right
"""
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np
import robosuite.utils.transform_utils as T
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

# Omega frame: x toward the operator, y to the operator's right, z up.
# Operator behind the robot (looking along robot +x): robot = (-x, -y, z).
_OMEGA_TO_BASE_BEHIND = np.diag([-1.0, -1.0, 1.0])
# Operator facing the robot (robot +x points at the operator): frames coincide.
_OMEGA_TO_BASE_FACING = np.eye(3)


def _ensure_libdrd_findable(logger) -> None:
    """forcedimension_core only looks in $FDSDK/lib/release/lin-<machine>-gcc/,
    ~/.local/lib and /usr/local/lib, and fails at import otherwise. If the SDK sits
    elsewhere (nested sdk-x.y.z/ folder, other folder naming, FDSDK unset), find libdrd
    and link it to ~/.local/lib/libdrd.so."""
    import glob
    import os
    import platform

    sdk = os.environ.get("FDSDK")
    standard = [
        os.path.expanduser("~/.local/lib/libdrd.so*"),
        "/usr/local/lib/libdrd.so*",
    ]
    if sdk:
        standard.insert(0, f"{sdk}/lib/release/lin-{platform.machine()}-gcc/libdrd.so.*")
    if any(glob.glob(p) for p in standard):
        return

    roots = [r for r in (sdk, str(Path(__file__).parent.parent / "third_party")) if r]
    found = sorted(
        f
        for r in roots
        for f in glob.glob(f"{r}/**/libdrd.so*", recursive=True)
        if os.path.isfile(f)
    )
    # SDK archives can ship several architectures -- prefer this machine's.
    found = [f for f in found if platform.machine() in f] or found
    if not found:
        raise RuntimeError(
            f"libdrd not found (FDSDK={sdk!r}, searched {roots}). Extract the Force "
            "Dimension SDK to third_party/forcedimension_sdk/ in the repo."
        )
    link = os.path.expanduser("~/.local/lib/libdrd.so")
    os.makedirs(os.path.dirname(link), exist_ok=True)
    if os.path.lexists(link):
        os.remove(link)
    os.symlink(found[-1], link)
    logger.info(f"Linked {found[-1]} -> {link}")


class _OmegaHaptics:
    """Owns all SDK calls. A ~1 kHz thread keeps sending zero force (the SDK adds gravity
    compensation on top) so the handle floats, and caches pose + button for the 20 Hz
    control loop."""

    def __init__(self, button_index: int, rate_hz: float, logger):
        _ensure_libdrd_findable(logger)
        from forcedimension_core import dhd, drd

        self._dhd, self._drd = dhd, drd
        self._button_index = button_index
        self._period = 1.0 / rate_hz

        if drd.open() < 0:
            raise RuntimeError(f"drd.open() failed: {dhd.errorGetLastStr()}")
        logger.info(f"Opened {dhd.getSystemName()}")

        if not drd.isInitialized():
            logger.info("Device not calibrated -- running drd.autoInit(), hands off the handle ...")
            if drd.autoInit() < 0:
                raise RuntimeError(f"drd.autoInit() failed: {dhd.errorGetLastStr()}")
        # Stop the DRD regulation thread but keep forces on, then drive forces via DHD.
        drd.stop(True)
        dhd.setGravityCompensation(True)
        if dhd.enableForce(True) < 0:
            raise RuntimeError(f"dhd.enableForce() failed: {dhd.errorGetLastStr()}")

        self._lock = threading.Lock()
        self._pos = None
        self._rot = None
        self._button = False
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        while self._pos is None:
            time.sleep(0.01)

    def _run(self) -> None:
        dhd = self._dhd
        pos = [0.0, 0.0, 0.0]
        mat = [[0.0, 0.0, 0.0] for _ in range(3)]
        zero = (0.0, 0.0, 0.0)
        while self._running:
            ok = dhd.getPositionAndOrientationFrame(pos, mat) >= 0
            dhd.setForce(zero)
            button = dhd.getButton(self._button_index) == 1
            if ok:
                with self._lock:
                    self._pos = np.array(pos)
                    self._rot = np.array(mat)
                    self._button = button
            time.sleep(self._period)

    def latest(self) -> Tuple[np.ndarray, np.ndarray, bool]:
        with self._lock:
            return self._pos.copy(), self._rot.copy(), self._button

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        self._dhd.enableForce(False)
        self._drd.close()


class _Omega6CartesianLeader(CartesianLeader):
    def __init__(
        self,
        gripper_button_index: int,
        translation_scale: float,
        rotation_scale: float,
        operator_facing_robot: bool,
        haptic_rate: float,
        **kwargs,
    ):
        self._translation_scale = translation_scale
        self._rotation_scale = rotation_scale
        self._map = _OMEGA_TO_BASE_FACING if operator_facing_robot else _OMEGA_TO_BASE_BEHIND
        self._prev_button = False
        self._gripper_closed = False
        self._anchor = None  # (dev_pos0, dev_rot0, ee_pos0, ee_rot0), set on first tick

        self._haptics = _OmegaHaptics(
            gripper_button_index, haptic_rate, logger=_StdoutLogger()
        )
        super().__init__(**kwargs)

    def read_device(self) -> Tuple[np.ndarray, np.ndarray, bool, bool]:
        pos, rot, button = self._haptics.latest()

        if button and not self._prev_button:
            self._gripper_closed = not self._gripper_closed
        self._prev_button = button

        if self._anchor is None:
            self._anchor = (pos, rot, self.ee_pos.copy(), self.ee_rot.copy())
        dev_pos0, dev_rot0, ee_pos0, ee_rot0 = self._anchor

        M = self._map
        target_pos = ee_pos0 + self._translation_scale * M @ (pos - dev_pos0)

        # Handle rotation since startup, expressed in the robot base frame, applied to the
        # startup end-effector orientation.
        d_rot = M @ (rot @ dev_rot0.T) @ M.T
        d_aa = T.quat2axisangle(T.mat2quat(d_rot)) * self._rotation_scale
        target_rot = T.quat2mat(T.axisangle2quat(d_aa)) @ ee_rot0

        dpos = target_pos - self.ee_pos
        drot = T.quat2axisangle(T.mat2quat(target_rot @ self.ee_rot.T))
        return dpos, drot, self._gripper_closed, False

    def destroy_node(self):
        self._haptics.close()
        super().destroy_node()


class _StdoutLogger:
    # The ROS node (and its logger) only exists after super().__init__, which needs the
    # device to be open already.
    def info(self, msg: str) -> None:
        print(f"[omega6] {msg}", flush=True)


@dataclass
class Args:
    side: str = "left"  # left, right, sim_left, sim_right
    control_freq: float = 20.0
    # Robot metres per handle metre / robot radians per handle radian.
    translation_scale: float = 1.0
    rotation_scale: float = 1.0
    # Max end-effector step per control tick -> max speed = limit * control_freq.
    ik_pos_limit: float = 0.02
    ik_ori_limit: float = 0.05
    gripper_button_index: int = 0
    # False: operator stands behind the robot, looking the same way it does.
    operator_facing_robot: bool = False
    haptic_rate: float = 1000.0


def main(args: Args) -> None:
    if args.side not in _ZMQ_ADDRESSES:
        raise ValueError(f"Invalid side '{args.side}'. Expected one of {list(_ZMQ_ADDRESSES)}.")

    leader = _Omega6CartesianLeader(
        gripper_button_index=args.gripper_button_index,
        translation_scale=args.translation_scale,
        rotation_scale=args.rotation_scale,
        operator_facing_robot=args.operator_facing_robot,
        haptic_rate=args.haptic_rate,
        name=args.side,
        zmq_addresses=_ZMQ_ADDRESSES[args.side],
        control_freq=args.control_freq,
        ik_pos_limit=args.ik_pos_limit,
        ik_ori_limit=args.ik_ori_limit,
        node_name=f"omega6_leader_{args.side}",
    )
    spin(leader)


if __name__ == "__main__":
    import rclpy

    rclpy.init()
    main(tyro.cli(Args))
