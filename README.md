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
│   ├── factr_teleop.py             # ROS2 launch file for the leader (FACTR teleop node)
│   ├── franka_ros2_follower.py     # CLI: real-robot follower bridge (single arm)
│   ├── franka_dual_arm.launch.py   # ROS2 launch file: two namespaced franka.launch.py (bimanual hardware)
│   ├── robosuite_sim.py            # CLI: robosuite follower sim (single arm or bimanual)
│   ├── start_real_robot_single_teleop.sh  # one-command tmux launcher, single-arm real robot
│   ├── start_bridge_sequence.sh    # helper called by the script above
│   ├── collect_data.py             # behavior-cloning data collection
│   ├── rollout.py                  # policy rollout
│   └── read_franka_q_offset.py     # small debug helper (leader/follower joint offset readout)
├── src/
│   ├── factr/
│   │   ├── factr_teleop/           # leader-side teleop node (ROS2 package "factr_teleop")
│   │   │   └── factr_teleop/configs/  # per-side YAML configs (franka_left.yaml, franka_right.yaml, franka_sim_left.yaml, franka_sim_right.yaml)
│   │   ├── follower_robots/        # follower bridges: franka_ros2_follower.py (real), robosuite_franka_follower.py (sim)
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

Then:
1. Install Docker Engine and the VS Code Dev Containers extension.
2. Open the `factr` directory in VS Code and "Rebuild and Reopen in Container". This builds the image and runs `post_create.sh` (clones `franka_ros2`, installs the Dynamixel SDK, runs `colcon build`).

The container is pinned to these versions for Franka Robot System **5.2.7**:

| Package | Version |
|---|---|
| `franka_ros2` | v0.1.0 |
| `libfranka` | 0.10.0 |
| ROS2 Humble | July 2023 snapshot |

If your robot runs a different system version, update all three before building:
1. [libfranka version for your Franka System version](https://frankarobotics.github.io/docs/doc/libfranka/docs/compatibility_matrix.html)
2. [franka_ros2 version for that libfranka version](https://frankarobotics.github.io/docs/doc/franka_ros2_humble/franka_ros2/doc/compatibility_matrix.html)
3. Match the ROS2 Humble snapshot date in the `Dockerfile` to that `franka_ros2` release date ([franka_ros2 tags](https://github.com/frankarobotics/franka_ros2/tags))

`robosuite` (and the compatible `mujoco`/`numpy` versions it needs — see [Troubleshooting](#troubleshooting)) is installed via `requirements.txt` as part of the image build.

## Getting Started

Try the simulation before touching real hardware.

### Launch Robosuite Simulation

Single arm, in a fresh terminal inside the container:
```bash
python launch/robosuite_sim.py --side left --table-offset-z 1.3
```

Bimanual (one process drives both arms — a shared physics step is required):
```bash
python launch/robosuite_sim.py --side both --table-offset-z 1.3
```

`--table-offset-z` raises the table (meters) relative to the robot base; omit it to use robosuite's default. Run `python launch/robosuite_sim.py --help` for all options (env name, controller gains, torque-feedback filtering, renderer).

Then launch the matching leader(s) — see [Launch FACTR Teleoperation](#launch-factr-teleoperation) with `side:=sim_left` / `side:=sim_right`.

## Launch Real Robot System

> **Safety:** the Franka system enforces collision/reflex thresholds and stops the robot when they're exceeded. Raising them is covered in [Troubleshooting](#increasing-franka-robot-system-thresholds-with-franka_ros2-v010).

### Single Arm — One-Command Script

```bash
bash launch/start_real_robot_single_teleop.sh left
```
This kills leftover processes, launches `franka.launch.py`, spawns `joint_trajectory_controller`, moves the arm to `[0, 0, 0, -1.57, 0, 1.57, 0.785]`, then starts the `franka_ros2` bridge — all inside `tmux`.

Add `True` to enable the bounding-box safety mode (follower stops once it leaves a predefined workspace box, defined in `src/factr/follower_robots/franka_ros2_follower.py`):
```bash
bash launch/start_real_robot_single_teleop.sh left True
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
python launch/franka_ros2_follower.py --side <side> --config-file <config_file> --save-launch
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
   python launch/franka_ros2_follower.py --side left \
       --config-file franka_left.yaml \
       --trajectory-topic /left/joint_trajectory_controller/joint_trajectory \
       --robot-state-topic /left/franka_robot_state_broadcaster/robot_state

   python launch/franka_ros2_follower.py --side right \
       --config-file franka_right.yaml \
       --trajectory-topic /right/joint_trajectory_controller/joint_trajectory \
       --robot-state-topic /right/franka_robot_state_broadcaster/robot_state
   ```

## Launch FACTR Teleoperation

Launch the leader for each side in its own terminal, matching whatever follower/simulation is already running. Controller behavior (gravity compensation, friction compensation, etc.) and per-side hardware parameters live in `src/factr/factr_teleop/factr_teleop/configs/`.

```bash
ros2 launch launch/factr_teleop.py side:=<side>
```
`<side>` ∈ `left`, `right`, `sim_left`, `sim_right` — this also selects the config file (`franka_<side>.yaml`).

## Troubleshooting

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
