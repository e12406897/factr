<h1>FACTR Teleop with Manipulator Redundancy: Low-Cost Force-Feedback Teleoperation</h1>

This repo extends [FACTR](https://github.com/RaindragonD/factr/) teleoperation for redundant robots, exploiting the manipulator's null-space motion to maximize haptic feedback for minimal joint torque.

[Project Page](https://jasonjzliu.com/factr/) | [arXiv](https://arxiv.org/abs/2502.17432) | [FACTR](https://github.com/RaindragonD/factr/) | [FACTR Hardware](https://github.com/JasonJZLiu/FACTR_Hardware)

On top of the original FACTR teleop code, this repo adds:
- a `franka_ros2` bridge for driving real Franka arms (single-arm and bimanual),
- a `robosuite`-based simulation for single-arm and bimanual setups.

## Catalog
- [Repository Overview](#repository-overview)
- [Installation](#installation)
- [Launch Robosuite Simulation](#launch-robosuite-simulation)
- [Launch Real Robot System](#launch-real-robot-system)
- [Launch FACTR Teleoperation](#launch-factr-teleoperation)
- [Troubleshooting](#troubleshooting)

## Repository Overview

```
factr/
├── .devcontainer/          # Dockerfile config, entrypoint, container setup script
├── launch/                 # Everything you actually run (Python + bash + ROS2 launch files)
│   ├── teleop_factr.py             # ROS2 launch file for the leader (FACTR teleop node)
│   ├── teleop_spacemouse.py        # CLI: 3Dconnexion SpaceMouse Cartesian leader
│   ├── teleop_omega6.py            # CLI: Force Dimension Omega.6 Cartesian leader
│   ├── franka_single_arm.launch.py # CLI: real-robot follower bridge (single arm)
│   ├── franka_dual_arm.launch.py   # ROS2 launch file: two namespaced franka.launch.py (bimanual hardware)
│   ├── franka_robosuite_sim.launch.py  # CLI: robosuite follower sim (single arm or bimanual)
│   ├── start_real_robot_single_teleop.sh  # one-command tmux launcher, single-arm real robot
│   ├── start_bridge_sequence.sh    # helper called by the script above
│   ├── collect_data.py             # behavior-cloning data collection
│   ├── rollout.py                  # policy rollout
│   └── read_offset.py              # small debug helper (leader/follower joint offset readout)
├── src/
│   ├── factr/
│   │   ├── factr_teleop/           # leader-side teleop node (ROS2 package "factr_teleop")
│   │   │   └── factr_teleop/configs/  # per-side YAML configs (franka_left.yaml, franka_right.yaml, franka_sim_left.yaml, franka_sim_right.yaml)
│   │   ├── follower_robots/        # follower bridges: franka_ros2_follower.py (real), robosuite_franka_follower.py (sim)
│   │   ├── cartesian_teleop/       # cartesian_leader.py: shared base for Cartesian-space leaders (SpaceMouse, Omega.6)
│   │   ├── python_utils/           # shared ZMQ messenger + global ZMQ address/config table
│   │   ├── bc/                     # behavior cloning: data recording, rollout
│   │   └── cameras/                # camera drivers/utilities
│   └── franka_ros2/                # third-party, git-cloned by post_create.sh (not committed)
├── requirements.txt         # Python deps installed into the container image
└── Dockerfile
```

**Leader ↔ follower communication** always uses ZMQ (see `src/factr/python_utils/python_utils/global_configs.py` for the address table), regardless of whether the follower is real hardware or `robosuite`. The gripper command/feedback channel is a plain ROS2 topic instead (`/factr_teleop/<side>/cmd_gripper_pos`). Leader and follower of one side must always run in the same ZMQ network namespace — see the [Troubleshooting](#troubleshooting) note on `franka_bridge_loopback_ip`.

Each side (`left`, `right`, `sim_left`, `sim_right`) has its own YAML config under `src/factr/factr_teleop/factr_teleop/configs/`. It sets the Dynamixel ports/servo types, joint signs, calibration pose, controller gains, and gripper actuation range for that specific leader arm.

## Installation

Before building the dev container, connect the teleoperation hardware:
1. **Check motor voltage before plugging in** — the wrong voltage can permanently damage a Dynamixel motor.
2. Connect the power hub boards to your PC, then verify they are detected:
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
   ```
3. When first setting up the hardware, motors need to be set correctly via Dynamixel Wizard 2.0 Software. All motors need to be set to baudrate of 4 million. In general motors of the same kind should have the same settings. This can be checked with 
   ```bash
   python launch/read_motor_specs.py
   ```
   Adjust the USB serial ports to correctly read them out. 
4. All serial ports need to have their latency timer set to 1 ms. Do that for each one with
   ```bash
   echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB<NUMBER_OF_PORT>/latency_timer
   ```
Then:
1. Install Docker Engine and the VS Code Dev Containers extension.
2. Open the `factr` directory in VS Code and "Rebuild and Reopen in Container". This builds the image and runs `post_create.sh` (clones `franka_ros2`, installs the Dynamixel SDK, runs `colcon build`).

The container is pinned to these versions for Franka Robot System **5.2.7**:

| Package | Version |
|---|---|
| `franka_ros2` | v0.1.0 |
| `libfranka` | 0.10.0 |
| ROS2 Humble | July 2023 snapshot |

`libfranka` is built from source in the `Dockerfile` (no host mount/install needed) — if your robot runs a different system version, update all three before building:
1. [libfranka version for your Franka System version](https://frankarobotics.github.io/docs/doc/libfranka/docs/compatibility_matrix.html), and change the `--branch 0.10.0` in the `Dockerfile`'s libfranka build step to match
2. [franka_ros2 version for that libfranka version](https://frankarobotics.github.io/docs/doc/franka_ros2_humble/franka_ros2/doc/compatibility_matrix.html), and update `FRANKA_ROS2_VERSION` in `.devcontainer/post_create.sh`
3. Match the ROS2 Humble snapshot date in the `Dockerfile` to that `franka_ros2` release date ([franka_ros2 tags](https://github.com/frankarobotics/franka_ros2/tags))

`robosuite` (and the compatible `mujoco`/`numpy` versions it needs — see [Troubleshooting](#troubleshooting)) is installed via `requirements.txt` as part of the image build.

## Getting Started

Try the simulation before touching real hardware.

### Launch Robosuite Simulation

Single arm, in a fresh terminal inside the container:
```bash
python launch/franka_robosuite_sim.launch.py --side left --table-offset-z 1.3
```

Bimanual (one process drives both arms — a shared physics step is required):
```bash
python launch/franka_robosuite_sim.launch.py --side both --table-offset-z 1.3
```

`--table-offset-z` raises the table (meters) relative to the robot base; omit it to use robosuite's default. Run `python launch/franka_robosuite_sim.launch.py --help` for all options (env name, controller gains, torque-feedback filtering, renderer).

Then launch the matching leader(s) — see [Launch FACTR Teleoperation](#launch-factr-teleoperation) with `side:=sim_left` / `side:=sim_right`.

## Launch Real Robot System

> **Safety:** the Franka system enforces collision/reflex thresholds and stops the robot when they're exceeded. Raising them is covered in [Troubleshooting](#increasing-franka-robot-system-thresholds-with-franka_ros2-v010).

### Single Arm — One-Command Script

```bash
bash launch/start_real_robot_single_teleop.sh left
```
This kills leftover processes, launches `franka.launch.py`, spawns `joint_trajectory_controller`, moves the arm to `[0, 0, 0, -1.57, 0, 1.57, 0.785]`, then starts the `franka_ros2` bridge — all inside `tmux`.

A 2nd argument overrides the initial pose the arm moves to (comma-separated joint positions in rad, no spaces, `j1,...,j7`, brackets optional) — defaults to Franka's home pose (`0,0,0,-1.57,0,1.57,0.785`) if omitted:
```bash
bash launch/start_real_robot_single_teleop.sh left 0,-0.5,0,-2.0,0,1.5,0.785
```

A 3rd argument overrides the Franka's hostname/IP (passed as `franka.launch.py`'s `robot_ip`) — defaults to `franka` if omitted:
```bash
bash launch/start_real_robot_single_teleop.sh left "" 192.168.1.5
```

Add `True` as the 4th argument to enable the bounding-box safety mode (follower stops once it leaves a predefined workspace box, defined in `src/factr/follower_robots/franka_ros2_follower.py`). Pass `""` for arguments you want to leave at their default:
```bash
bash launch/start_real_robot_single_teleop.sh left "" "" True
```

Switch between the `tmux` windows from a **second, fresh terminal** (the aliases are added to `.bashrc` by the script on first run):
```bash
sw-hw   # switch to factr:hardware window
sw-br   # switch to factr:bridge window
```

### Single Arm — Manual Commands

Use this if you need to run a step individually (e.g. after the one-shot script failed midway).

```bash
source /opt/ros/humble/setup.bash
source /factr/install/setup.bash
ros2 launch franka_bringup franka.launch.py robot_ip:=<robot_ip>
```

In a new terminal, spawn the trajectory controller:
```bash
ros2 run controller_manager spawner joint_trajectory_controller
```

Move the arm to a known pose before starting the bridge — either a specific pose:
```bash
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], points: [{positions: <POSITION>, time_from_start: {sec: 4, nanosec: 0}}]}"
```
or the factory home position:
```bash
ros2 launch franka_bringup move_to_start_example_controller.launch.py robot_ip:=<robot_ip>
```

Then start the `franka_ros2` bridge:
```bash
python launch/franka_single_arm.launch.py --side <side> --config-file <config_file> --save-launch
```
- `<side>`: `left` or `right`.
- `<config_file>`: the matching FACTR teleop config, e.g. `franka_left.yaml` — keeping this identical to the leader's config avoids parameter mismatches (e.g. gripper actuation range) between the two processes.
- `--save-launch`: enables the bounding-box safety mode (omit for unrestricted motion).

### Bimanual System

There is no one-command script for bimanual real hardware yet — launch each piece manually.

1. Bring up both arms, each pushed into its own ROS2 namespace (`/left`, `/right`):
   ```bash
   ros2 launch launch/franka_dual_arm.launch.py left_robot_ip:=<left_ip> right_robot_ip:=<right_ip>
   ```
2. Spawn the trajectory controller for each side:
   ```bash
   ros2 run controller_manager spawner joint_trajectory_controller --controller-manager /left/controller_manager
   ros2 run controller_manager spawner joint_trajectory_controller --controller-manager /right/controller_manager
   ```
3. Start one bridge process per side, pointing at the namespaced topics:
   ```bash
   python launch/franka_single_arm.launch.py --side left \
       --config-file franka_left.yaml \
       --trajectory-topic /left/joint_trajectory_controller/joint_trajectory \
       --robot-state-topic /left/franka_robot_state_broadcaster/robot_state

   python launch/franka_single_arm.launch.py --side right \
       --config-file franka_right.yaml \
       --trajectory-topic /right/joint_trajectory_controller/joint_trajectory \
       --robot-state-topic /right/franka_robot_state_broadcaster/robot_state
   ```

## Launch FACTR Teleoperation

Launch the leader for each side in its own terminal, matching whatever follower/simulation is already running. Controller behavior (gravity compensation, friction compensation, etc.) and per-side hardware parameters live in `src/factr/factr_teleop/factr_teleop/configs/`.

Advice for how configuration parameters are tuned see next subsection (e.g. fixing offset).

```bash
ros2 launch launch/teleop_factr.py side:=<side>
```
`<side>` ∈ `left`, `right`, `sim_left`, `sim_right` — this also selects the config file (`franka_<side>.yaml`).

When null space torque optimization is enabled it only enables enhancing manipulability first. By pressing F9 the feature can be activated. It should adjust its elbow position according to the external force maximizing so that the torque that needs to be applied against it is minimal.

Other commercial cartesian leader systems can also be launched with 

```bash
python launch/teleop_omega6.py --side <side> --translation_scale <float>
```

or with

```bash
python launch/teleop_spacemouse.py --side <side> 
```

### Tuning Configuration Parameters
In general simulation needs lower force feedback gains than in real world setups. Joints 2 and 4 need slightly smaller gains because of stronger motors. In general a feedback gain is chosen too high if jumpy behavior is seen in the joint when external force is applied. Higher gains can be chosen if the filter only slowly updates very slowly the external force signal, but the slow build up in the signal results in delayed haptic feeling. This is a trade off

Gravity compensation is not relevant for the first joint. It needs to be chosen way higher for the smaller motors than for joint 2 and 4. These are the key joints since they are responsible for holding the most weight. If the arm is pulling slowly in one direction then a gain might be chosen too high. Futhermore, gravity compensation introduces a offset between the leader and follower. There is a helper script (only for single-arm setup yet) within the launch directory. While the teleoperation is running, execute in a fresh terminal

```bash
python launch/franka_offset
```

and change the joint_bias_correction - values respectively. This adds a constant offset to the read motor position signals. Note that the last joint needs to have a offset of 0.785 to match the real world position, so the offset needs to be added to this value. Test this in simulation and figure out the correct offset to prevent any damage in real world. 

Friction compensation gains need to be increased until a constant vibration is felt when the system is in rest. This overcomes the static coloumb friction by never resting. If the system is constantly vibrating the enable_speed value can be lowered. When friction compensation is enabled it might again require adjustments within gravity compensation.

null space regularization is a normal PD controller scheme for each joint individually. null_space_joint_target gives the resting pose in there.

null space torque optimization has damping term as well. If null space motion is too aggressive increase values. Manipulability gains can be chosen quite high in comparison. 
With pressing F9 during operation enhancing torque optimization can be activated. The gains weight each dimension of the external torque individually. So a user can for example define to only react to external forces in z-direction by setting all other values to zero. The sign is defining if it rather avoiding singularity or enhancing torque optimization. Be careful with this feature. It is not tested properly tested and can be falsly implemented. Have a check.

The actuation range can be increased within the "gripper_teleop" section within the config file. There is also the possibility of activating gripper force feedback. 
(the gripper force feedback code was not tested for a while, because of binary control scheme was used. Can require some adjustments)


## Troubleshooting

### Weird Base-Link Motion
It can happen that the screws connecting Link 1 and the motor of the first joint loosen. This can result in weird movements when only one screw is connected. Test each joint following behavior separately and figure out where the problem is coming from. If it is the base link it is probably that. If it is fully disconnected there should be no motion encountered. 

On long term longer threadlock should be used and longer screws
### Increasing Franka Robot System Thresholds with franka_ros2 v0.1.0

`franka_ros2` v0.1.0 has no runtime API to raise collision/force thresholds. Patch them directly in `src/franka_ros2/franka_hardware/src/robot.cpp`, inside the `Robot` constructor, right after the `franka::Robot` connection is established:

```cpp
Robot::Robot(const std::string& robot_ip, const rclcpp::Logger& logger) {
  tau_command_.fill(0.);
  franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;
  if (!franka::hasRealtimeKernel()) {
    rt_config = franka::RealtimeConfig::kIgnore;
    RCLCPP_WARN(logger, "You are not using a real-time kernel...");
  }
  robot_ = std::make_unique<franka::Robot>(robot_ip, rt_config);

  // Raised from libfranka's conservative defaults to match our teleop workload.
  robot_->setCollisionBehavior(
      {{80, 80, 80, 80, 30, 30, 30}},         // lower_torque_thresholds_acceleration
      {{80, 80, 80, 80, 30, 30, 30}},         // upper_torque_thresholds_acceleration
      {{25, 25, 22, 20, 19, 17, 14}},         // lower_torque_thresholds_nominal
      {{100, 100, 100, 100, 100, 100, 100}},  // upper_torque_thresholds_nominal
      {{80, 80, 80, 30, 30, 30}},             // lower_force_thresholds_acceleration
      {{80, 80, 80, 30, 30, 30}},             // upper_force_thresholds_acceleration
      {{100, 100, 100, 100, 100, 100}},       // lower_force_thresholds_nominal
      {{100, 100, 100, 100, 100, 100}});      // upper_force_thresholds_nominal

  model_ = std::make_unique<franka::Model>(robot_->loadModel());
  franka_hardware_model_ = std::make_unique<Model>(model_.get());
}
```

Rebuild the package inside the running container to apply it:
```bash
colcon build --packages-select franka_hardware
```

### ZMQ bind address / port already in use
Leader and follower for one side communicate over `127.0.0.2` (see `franka_bridge_loopback_ip` in `src/factr/python_utils/python_utils/global_configs.py`) — not `127.0.0.1`. That's deliberate: VS Code's automatic port forwarding can latch onto a port once used on `127.0.0.1` and hold it open on the host even after the original process exits, causing `Address already in use` on the next launch. If you still hit that error, check the "PORTS" tab in VS Code's terminal panel and stop forwarding the affected port, or pick another loopback address (any `127.0.0.0/8` address works).

### Varying feedback between Dynamixel motors
Make via Dynamixel Wizard sure, that all motors have the same gains applied. Different gains can lead to different control characteristics 
