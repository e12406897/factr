import pickle
import threading
import time
from typing import Any, Dict, Optional

import mujoco
import mujoco.viewer
import numpy as np
import zmq
from dm_control import mjcf

from src.robots.robot import Robot

assert mujoco.viewer is mujoco.viewer


def attach_hand_to_arm(
    arm_mjcf: mjcf.RootElement,
    hand_mjcf: mjcf.RootElement,
) -> None:
    """Attaches a hand to an arm.

    The arm must have a site named "attachment_site".

    Taken from https://github.com/deepmind/mujoco_menagerie/blob/main/FAQ.md#how-do-i-attach-a-hand-to-an-arm

    Args:
      arm_mjcf: The mjcf.RootElement of the arm.
      hand_mjcf: The mjcf.RootElement of the hand.

    Raises:
      ValueError: If the arm does not have a site named "attachment_site".
    """
    physics = mjcf.Physics.from_mjcf_model(hand_mjcf)

    attachment_site = arm_mjcf.find("site", "attachment_site")
    if attachment_site is None:
        raise ValueError("No attachment site found in the arm model.")

    # Expand the ctrl and qpos keyframes to account for the new hand DoFs.
    arm_key = arm_mjcf.find("key", "home")
    if arm_key is not None:
        hand_key = hand_mjcf.find("key", "home")
        if hand_key is None:
            arm_key.ctrl = np.concatenate([arm_key.ctrl, np.zeros(physics.model.nu)])
            arm_key.qpos = np.concatenate([arm_key.qpos, np.zeros(physics.model.nq)])
        else:
            arm_key.ctrl = np.concatenate([arm_key.ctrl, hand_key.ctrl])
            arm_key.qpos = np.concatenate([arm_key.qpos, hand_key.qpos])

    attachment_site.attach(hand_mjcf)


def build_scene(robot_xml_path: str, gripper_xml_path: Optional[str] = None):
    # assert robot_xml_path.endswith(".xml")

    arena = mjcf.RootElement()
    arm_simulate = mjcf.from_path(robot_xml_path)
    # arm_copy = mjcf.from_path(xml_path)

    if gripper_xml_path is not None:
        # attach gripper to the robot at "attachment_site"
        gripper_simulate = mjcf.from_path(gripper_xml_path)
        attach_hand_to_arm(arm_simulate, gripper_simulate)

    # # Add wrist camera to the end effector
    # attachment_site = arm_simulate.find("site", "attachment_site")
    # if attachment_site is not None:
    #     # Add camera at the attachment site (wrist)
    #     attachment_site.parent.add(
    #         "camera",
    #         name="wrist_camera",
    #         pos=[0, 0, 0.05],  # 5cm above the attachment site
    #         quat=[0.707, 0.707, 0, 0],  # Looking forward and down
    #         fovy=60,  # Field of view
    #     )

    # Floor
    arena.worldbody.add(
        "geom",
        type="plane",
        size=[2, 2, 0.1],
        rgba=[0.5, 0.5, 0.5, 1],
        name="floor",
    )

    # Table: top surface at z=0.4, legs omitted for simplicity
    table_x, table_y, table_z = 0.5, 0.0, 0.4  # table top centre
    table_hw, table_hd, table_ht = 0.3, 0.3, 0.02  # half width/depth/thickness
    table_body = arena.worldbody.add(
        "body", name="table", pos=[table_x, table_y, table_z]
    )
    table_body.add(
        "geom",
        type="box",
        size=[table_hw, table_hd, table_ht],
        rgba=[0.6, 0.4, 0.2, 1],
        name="table_top",
    )

    # Hole: square socket flush on the table surface
    # 4 walls forming a square recess; peg must fit inside
    _hole_hw = 0.025  # inner half-width of the hole opening
    _wall_t = 0.005  # wall half-thickness
    _wall_h = 0.04  # socket depth half-height
    _wall_z = table_ht + _wall_h  # above table top
    hole_rgba = [0.3, 0.3, 0.8, 1]
    hole_body = arena.worldbody.add(
        "body", name="hole", pos=[table_x, table_y, table_z]
    )
    for name, pos, size in [
        (
            "hole_wall_n",
            [0, _hole_hw + _wall_t, _wall_z],
            [_hole_hw + _wall_t, _wall_t, _wall_h],
        ),
        (
            "hole_wall_s",
            [0, -(_hole_hw + _wall_t), _wall_z],
            [_hole_hw + _wall_t, _wall_t, _wall_h],
        ),
        (
            "hole_wall_e",
            [_hole_hw + _wall_t, 0, _wall_z],
            [_wall_t, _hole_hw + _wall_t, _wall_h],
        ),
        (
            "hole_wall_w",
            [-(_hole_hw + _wall_t), 0, _wall_z],
            [_wall_t, _hole_hw + _wall_t, _wall_h],
        ),
    ]:
        hole_body.add("geom", type="box", name=name, pos=pos, size=size, rgba=hole_rgba)

    # Peg: square box, starts on the table next to the hole
    peg_body = arena.worldbody.add(
        "body", name="peg", pos=[table_x - 0.15, table_y, table_z + table_ht + 0.04]
    )
    peg_body.add(
        "inertial", pos=[0, 0, 0], mass=0.1, diaginertia=[0.0001, 0.0001, 0.00005]
    )
    peg_body.add("joint", type="free", name="peg_joint")
    peg_body.add(
        "geom",
        type="box",
        size=[0.02, 0.02, 0.04],  # 4x4cm cross-section, 8cm tall
        rgba=[0.8, 0.3, 0.3, 1],
        name="peg_geom",
    )

    # Add a wall to the left side of the robot
    arena.worldbody.add(
        "geom",
        type="box",
        size=[0.01, 1.0, 0.75],  # thin (2cm), wide (2m), tall (1.5m)
        pos=[-0.3, 0.0, 0.75],
        rgba=[0.8, 0.8, 0.8, 1],
        name="wall",
    )

    arm_frame = arena.worldbody.attach(arm_simulate)
    # arena.worldbody.attach(arm_copy)

    return arena


class ZMQServerThread(threading.Thread):
    def __init__(self, server):
        super().__init__()
        self._server = server

    def run(self):
        self._server.serve()

    def terminate(self):
        self._server.stop()


class ZMQRobotServer:
    """A class representing a ZMQ server for a robot."""

    def __init__(self, robot: Robot, host: str = "127.0.0.1", port: int = 5556):
        self._robot = robot
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REP)
        addr = f"tcp://{host}:{port}"
        self._socket.bind(addr)
        self._stop_event = threading.Event()

    def serve(self) -> None:
        """Serve the robot state and commands over ZMQ."""
        self._socket.setsockopt(zmq.RCVTIMEO, 1000)  # Set timeout to 1000 ms
        while not self._stop_event.is_set():
            try:
                message = self._socket.recv()
                request = pickle.loads(message)

                # Call the appropriate method based on the request
                method = request.get("method")
                args = request.get("args", {})
                result: Any
                if method == "num_dofs":
                    result = self._robot.num_dofs()
                elif method == "get_joint_state":
                    result = self._robot.get_joint_state()
                elif method == "command_joint_state":
                    result = self._robot.command_joint_state(**args)
                elif method == "get_observations":
                    result = self._robot.get_observations()
                else:
                    result = {"error": "Invalid method"}
                    print(result)
                    raise NotImplementedError(
                        f"Invalid method: {method}, {args, result}"
                    )

                self._socket.send(pickle.dumps(result))
            except zmq.error.Again:
                print("Timeout in ZMQLeaderServer serve")
                # Timeout occurred, check if the stop event is set

    def stop(self) -> None:
        self._stop_event.set()
        self._socket.close()
        self._context.term()


class MujocoRobotServer:
    def __init__(
        self,
        xml_path: str,
        gripper_xml_path: Optional[str] = None,
        host: str = "127.0.0.1",
        port: int = 5556,
        print_joints: bool = False,
    ):
        self._has_gripper = gripper_xml_path is not None
        arena = build_scene(xml_path, gripper_xml_path)

        assets: Dict[str, str] = {}
        for asset in arena.asset.all_children():
            if asset.tag == "mesh":
                f = asset.file
                assets[f.get_vfs_filename()] = asset.file.contents

        xml_string = arena.to_xml_string()
        # save xml_string to file
        with open("arena.xml", "w") as f:
            f.write(xml_string)

        self._model = mujoco.MjModel.from_xml_string(xml_string, assets)
        self._data = mujoco.MjData(self._model)

        self._num_joints = self._model.nu

        self._joint_state = np.zeros(self._num_joints)
        self._joint_cmd = self._joint_state

        self._zmq_server = ZMQRobotServer(robot=self, host=host, port=port)
        self._zmq_server_thread = ZMQServerThread(self._zmq_server)

        self._print_joints = print_joints

    def num_dofs(self) -> int:
        return self._num_joints

    def get_joint_state(self) -> np.ndarray:
        return self._joint_state

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        assert len(joint_state) == self._num_joints, (
            f"Expected joint state of length {self._num_joints}, "
            f"got {len(joint_state)}."
        )
        if self._has_gripper:
            _joint_state = joint_state.copy()
            _joint_state[-1] = _joint_state[-1] * 255
            self._joint_cmd = _joint_state
        else:
            _joint_state = joint_state.copy()
            _joint_state[-1] = _joint_state[-1] * 255
            self._joint_cmd = _joint_state

    def freedrive_enabled(self) -> bool:
        return True

    def set_freedrive_mode(self, enable: bool):
        pass

    def get_observations(self) -> Dict[str, np.ndarray]:
        joint_positions = self._data.qpos.copy()[: self._num_joints]
        joint_velocities = self._data.qvel.copy()[: self._num_joints]
        ee_site = "attachment_site"
        try:
            ee_pos = self._data.site_xpos.copy()[
                mujoco.mj_name2id(self._model, 6, ee_site)
            ]
            ee_mat = self._data.site_xmat.copy()[
                mujoco.mj_name2id(self._model, 6, ee_site)
            ]
            ee_quat = np.zeros(4)
            mujoco.mju_mat2Quat(ee_quat, ee_mat)
        except Exception:
            ee_pos = np.zeros(3)
            ee_quat = np.zeros(4)
            ee_quat[0] = 1
        gripper_pos = self._data.qpos.copy()[self._num_joints - 1]
        # import pdb

        # pdb.set_trace()
        joint_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_JOINT, "fr3/fr3_joint1"
        )
        dof_start = self._model.jnt_dofadr[joint_id]  # = 6, not 0
        return {
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "ee_pos_quat": np.concatenate([ee_pos, ee_quat]),
            "gripper_position": gripper_pos,
            "qfrc_actuator": self._data.qfrc_actuator.copy()[
                dof_start : dof_start + self._num_joints
            ],
            "qfrc_constraint": self._data.qfrc_constraint.copy()[
                dof_start : dof_start + self._num_joints
            ],
        }

    def serve(self) -> None:
        # start the zmq server
        self._zmq_server_thread.start()
        with mujoco.viewer.launch_passive(self._model, self._data) as viewer:
            while viewer.is_running():
                step_start = time.time()

                # mj_step can be replaced with code that also evaluates
                # a policy and applies a control signal before stepping the physics.
                self._data.ctrl[:] = self._joint_cmd
                # self._data.qpos[:] = self._joint_cmd
                mujoco.mj_step(self._model, self._data)
                self._joint_state = self._data.qpos.copy()[: self._num_joints]

                if self._print_joints:
                    print(self._joint_state)

                # Example modification of a viewer option: toggle contact points every two seconds.
                with viewer.lock():
                    # TODO remove?
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(
                        self._data.time % 2
                    )

                # Pick up changes to the physics state, apply perturbations, update options from GUI.
                viewer.sync()

                # Rudimentary time keeping, will drift relative to wall clock.
                time_until_next_step = self._model.opt.timestep - (
                    time.time() - step_start
                )
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    def stop(self) -> None:
        self._zmq_server_thread.join()

    def __del__(self) -> None:
        self.stop()
