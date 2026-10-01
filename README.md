<div align="center">

# 🤖 Bumperbot Workspace (`bumperbot_ws`)

**A ROS 2 Jazzy workspace for a differential-drive mobile robot, with its URDF/Xacro model, Gazebo simulation, `ros2_control` drive stack, joystick teleoperation, and a from-scratch kinematics controller written in Python.**

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Simulator-Gazebo%20Sim-F58113)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)
![Build](https://img.shields.io/badge/build-colcon-informational)

</div>

---

## Table of Contents

1. [Overview](#1-overview)
2. [Feature Summary](#2-feature-summary)
3. [Complete Workspace Tree](#3-complete-workspace-tree)
4. [System Architecture](#4-system-architecture)
5. [Package-by-Package Breakdown](#5-package-by-package-breakdown)
6. [The Robot Model](#6-the-robot-model)
7. [Controllers and Kinematics](#7-controllers-and-kinematics)
8. [Prerequisites](#8-prerequisites)
9. [Build](#9-build)
10. [Run](#10-run)
11. [ROS Interfaces Reference](#11-ros-interfaces-reference)
12. [Configuration Reference](#12-configuration-reference)
13. [Practice Nodes (`bumperbot_py_examples`)](#13-practice-nodes-bumperbot_py_examples)
14. [Git Workflow (`gitsync.sh`)](#14-git-workflow-gitsyncsh)
15. [Troubleshooting](#15-troubleshooting)
16. [Credits and License](#16-credits-and-license)

---

## 1. Overview

**Bumperbot** is a small two-wheeled, differential-drive robot with two passive casters. This workspace contains everything needed to model it, simulate it, and drive it:

- A **URDF/Xacro** description with real STL meshes, inertial properties and collision geometry.
- A **Gazebo Sim** launch setup that spawns the robot into an empty world.
- A **`ros2_control`** hardware interface exposing both wheel joints as velocity-controlled joints.
- **Two interchangeable drive controllers**: a custom Python kinematics node (`simple_controller`) and the stock `diff_drive_controller/DiffDriveController`.
- **Joystick teleoperation** through `joy` and `joy_teleop`.
- A set of **`rclpy` practice nodes** (pub/sub, parameters, services, TF, turtlesim kinematics) used to build up the fundamentals.

The workspace targets **ROS 2 Jazzy** on **Ubuntu 24.04**. Launch files also detect `humble` and switch to the Ignition plugins automatically.

---

## 2. Feature Summary

| Area | What is implemented |
| --- | --- |
| **Modelling** | Xacro-based URDF with `base_footprint → base_link`, two continuous wheel joints, two fixed casters, inertias, and mesh visuals |
| **Simulation** | Gazebo Sim (`gz_sim`) launch with `-r empty.sdf`, robot spawned from `/robot_description` |
| **Hardware abstraction** | `ros2_control` system with velocity command interfaces (clamped to ±1) and position and velocity state interfaces |
| **Custom controller** | Python node that converts `TwistStamped` into left/right wheel velocities through an inverse differential-drive matrix |
| **Stock controller** | `DiffDriveController` with velocity and acceleration limits, covariance settings, odometry and odom TF |
| **Teleop** | Gamepad with a deadman button, mapped to linear X and angular Z |
| **Distro awareness** | Launch files choose Gazebo Sim or Ignition plugins based on `$ROS_DISTRO` |
| **Learning nodes** | 7 `rclpy` examples plus a custom `AddTwoInts` service interface |

---

## 3. Complete Workspace Tree

Only files tracked in Git are shown. `build/`, `install/` and `log/` are produced by `colcon build` and are ignored via `.gitignore`.

```text
bumperbot_ws/
├── .gitignore                         # Ignores colcon output and the 1 GB+ VS Code IntelliSense cache
├── README.md                          # This file
├── gitsync.sh                         # One-command "add, commit and push" helper
│
├── .vscode/
│   ├── c_cpp_properties.json          # C/C++ IntelliSense configuration for ROS 2
│   └── settings.json                  # ROS2 distro = jazzy, Python analysis paths
│
└── src/
    │
    ├── bumperbot_description/         # ── ROBOT MODEL (ament_cmake) ──
    │   ├── CMakeLists.txt             # Installs urdf/, meshes/, launch/, rviz/
    │   ├── package.xml
    │   ├── launch/
    │   │   ├── display.launch.py      # RViz + robot_state_publisher + joint_state_publisher
    │   │   └── gazebo.launch.py       # Gazebo Sim + robot_state_publisher + entity spawn
    │   ├── meshes/
    │   │   ├── base_link.STL          # Chassis (≈13.6 MB)
    │   │   ├── wheel_left_link.STL    # Left wheel
    │   │   ├── wheel_right_link.STL   # Right wheel
    │   │   ├── caster_front_link.STL  # Front caster
    │   │   ├── caster_rear_link.STL   # Rear caster
    │   │   ├── imu_link.STL           # IMU mesh (not yet attached in the URDF)
    │   │   └── can.STL                # Spare mesh (not yet attached in the URDF)
    │   ├── rviz/
    │   │   └── display.rviz           # Saved RViz layout
    │   └── urdf/
    │       ├── bumperbot.urdf.xacro           # Main model (links, joints, inertias)
    │       ├── bumperbot_gazebo.xacro         # Gazebo friction/contact tags + control plugin
    │       └── bumperbot_ros2_control.xacro   # ros2_control hardware and joint interfaces
    │
    ├── bumperbot_controller/          # ── DRIVE CONTROL (ament_cmake + Python) ──
    │   ├── CMakeLists.txt             # Installs launch/, config/ and simple_controller.py
    │   ├── package.xml
    │   ├── bumperbot_controller/
    │   │   ├── __init__.py
    │   │   └── simple_controller.py   # Custom differential-drive kinematics node
    │   ├── config/
    │   │   ├── bumperbot_controllers.yaml  # controller_manager + DiffDrive + velocity controller
    │   │   ├── joy_config.yaml             # joy_node parameters
    │   │   └── joy_teleop.yaml             # Gamepad → TwistStamped mapping
    │   └── launch/
    │       ├── controller.launch.py        # Spawns controllers, selects custom or stock
    │       └── joystick_teleop.launch.py   # joy_node + joy_teleop
    │
    ├── bumperbot_msgs/                # ── CUSTOM INTERFACES (ament_cmake + rosidl) ──
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── srv/
    │       └── AddTwoInts.srv         # int64 a, b → int64 sum
    │
    ├── bumperbot_py_examples/         # ── rclpy PRACTICE NODES (ament_python) ──
    │   ├── package.xml
    │   ├── setup.cfg
    │   ├── setup.py                   # Console-script entry points
    │   ├── resource/
    │   │   └── bumperbot_py_examples  # ament index marker
    │   ├── bumperbot_py_examples/
    │   │   ├── __init__.py
    │   │   ├── simple_publisher.py            # Publishes String on /chatter at 1 Hz
    │   │   ├── simple_subscriber.py           # Subscribes to /chatter
    │   │   ├── simple_parameter.py            # Parameters with validation callback
    │   │   ├── simple_service_server.py       # add_two_ints server
    │   │   ├── simple_service_client.py       # add_two_ints client (CLI args)
    │   │   ├── simple_tf_kinematics.py        # Static + dynamic TF broadcasting
    │   │   └── simple_turtlesim_kinematics.py # Relative pose of turtle2 w.r.t. turtle1
    │   └── test/
    │       ├── test_copyright.py
    │       ├── test_flake8.py
    │       └── test_pep257.py
    │
    └── bumperbot_cpp_examples/        # ── rclcpp SCAFFOLD (ament_cmake) ──
        ├── CMakeLists.txt
        └── package.xml
```

---

## 4. System Architecture

### TF tree

```text
odom
 └── base_footprint            (published by DiffDriveController when enable_odom_tf = true)
      └── base_link            (fixed, z = 0.033 m)
           ├── wheel_left_link     (continuous, y = +0.0701 m)
           ├── wheel_right_link    (continuous, y = −0.0701 m)
           ├── caster_front_link   (fixed, x = +0.04755 m, z = −0.0275 m)
           └── caster_rear_link    (fixed, x = −0.04755 m, z = −0.0275 m)
```

---

## 5. Package-by-Package Breakdown

### 5.1 `bumperbot_description`

Everything about what the robot **is**.

| File | Role |
| --- | --- |
| `urdf/bumperbot.urdf.xacro` | Main model. Declares the `is_ignition` argument and includes the two helper Xacro files. Defines all links, joints, masses and inertia tensors. |
| `urdf/bumperbot_gazebo.xacro` | Gazebo-specific contact tuning (`mu1`, `mu2`, `kp`, `kd`, `minDepth`, `maxVel`) per link, plus the `ros2_control` Gazebo plugin loading `bumperbot_controllers.yaml`. Selects `gz_ros2_control` or `ign_ros2_control` through `is_ignition`. |
| `urdf/bumperbot_ros2_control.xacro` | `ros2_control` system description. Both wheel joints get a `velocity` command interface (min −1, max 1) and `position` and `velocity` state interfaces. |
| `launch/display.launch.py` | Runs `xacro` on the model, starts `robot_state_publisher`, `joint_state_publisher` and `rviz2` with the saved config. Accepts a `model:=` argument. |
| `launch/gazebo.launch.py` | Runs `xacro` with the right `is_ignition` value, sets `GZ_SIM_RESOURCE_PATH` so meshes resolve, starts Gazebo Sim with `-v 4 -r empty.sdf`, and spawns the robot as `bumperbot` from the `robot_description` topic. |
| `meshes/*.STL` | Visual meshes. Collision uses simple spheres (wheels r = 0.033 m, casters r = 0.005 m) for fast, stable physics. |
| `rviz/display.rviz` | Pre-configured RViz displays. |

### 5.2 `bumperbot_controller`

Everything about how the robot **moves**.

| File | Role |
| --- | --- |
| `bumperbot_controller/simple_controller.py` | Custom node implementing the differential-drive kinematic model (details in [section 7](#7-controllers-and-kinematics)). |
| `config/bumperbot_controllers.yaml` | Defines the controller manager at 100 Hz, registers `bumperbot_controller` (DiffDrive), `joint_state_broadcaster` and `simple_velocity_controller`, and holds all of their parameters. |
| `config/joy_config.yaml` | `joy_node` settings: device 0, deadzone 0.5, autorepeat 20 Hz. |
| `config/joy_teleop.yaml` | Maps joystick axes and a deadman button to `TwistStamped` commands. |
| `launch/controller.launch.py` | Spawns the broadcaster and the chosen controller. Arguments in [section 12](#12-configuration-reference). |
| `launch/joystick_teleop.launch.py` | Starts `joy_node` and `joy_teleop` with the YAML configs above. |

### 5.3 `bumperbot_msgs`

Custom interface package. Currently provides `AddTwoInts.srv` (`int64 a`, `int64 b` → `int64 sum`), generated with `rosidl_default_generators` and used by the service examples.

### 5.4 `bumperbot_py_examples`

An `ament_python` package of small, focused `rclpy` nodes covering core ROS 2 concepts. See [section 13](#13-practice-nodes-bumperbot_py_examples).

### 5.5 `bumperbot_cpp_examples`

An `ament_cmake` scaffold ready for `rclcpp` nodes. It builds, but contains no source files yet.

---

## 6. The Robot Model

### 6.1 Links and joints

| Link | Parent joint | Type | Origin relative to parent |
| --- | --- | --- | --- |
| `base_footprint` | none (root) | n/a | n/a |
| `base_link` | `base_joint` | fixed | z = 0.033 m |
| `wheel_right_link` | `wheel_right_joint` | continuous (axis Y) | y = −0.0701 m |
| `wheel_left_link` | `wheel_left_joint` | continuous (axis Y) | y = +0.0701 m |
| `caster_front_link` | `caster_front_joint` | fixed | x = +0.04755 m, z = −0.0275 m |
| `caster_rear_link` | `caster_rear_joint` | fixed | x = −0.04755 m, z = −0.0275 m |

### 6.2 Key physical parameters

| Parameter | Value |
| --- | --- |
| Chassis mass | ≈ 0.826 kg |
| Wheel mass (each) | ≈ 0.053 kg |
| Wheel radius | 0.033 m |
| Wheel separation | 0.17 m in the controller config (the URDF wheel joints sit ≈ 0.1402 m apart, see the note below) |
| Wheel friction (Gazebo) | `mu1 = mu2 = 1e14` (effectively no slip) |
| Caster friction (Gazebo) | `mu1 = mu2 = 0.1` (low friction, so casters slide freely) |

> ⚠️ The URDF wheel joints are about 0.1402 m apart while the controllers are configured with `wheel_separation = 0.17`. If odometry drifts in turns, compare these two numbers first.

---

## 7. Controllers and Kinematics

### 7.1 Differential-drive model

For wheel radius **r**, wheel separation **L**, left/right wheel angular velocities **ω<sub>L</sub>**, **ω<sub>R</sub>**:

```text
v = r/2 · (ω_R + ω_L)        # linear velocity   [m/s]
ω = r/L · (ω_R − ω_L)        # angular velocity  [rad/s]
```

In matrix form:

```text
┌ v ┐   ┌  r/2    r/2 ┐ ┌ ω_R ┐
│   │ = │             │ │     │
└ ω ┘   └  r/L   −r/L ┘ └ ω_L ┘
```

`simple_controller.py` builds this matrix once, then for every incoming command computes `[ω_R, ω_L] = M⁻¹ · [v, ω]` and publishes the result in joint order `[left, right]`.

### 7.2 Controller modes

| Mode | Launch argument | Command topic | How it drives the wheels |
| --- | --- | --- | --- |
| **Custom** (default) | `use_simple_controller:=True` | `bumperbot_controller/cmd_vel` (`TwistStamped`) | `simple_controller` → `simple_velocity_controller/commands` → `JointGroupVelocityController` |
| **Stock** | `use_simple_controller:=False` | `bumperbot_controller/cmd_vel` (`TwistStamped`) | `DiffDriveController` with limits, odometry and TF |

Stock controller limits (from `bumperbot_controllers.yaml`):

| Axis | Max velocity | Min velocity | Max acceleration | Min acceleration |
| --- | --- | --- | --- | --- |
| Linear X | 1.0 m/s | −0.5 m/s | 0.8 m/s² | −0.4 m/s² |
| Angular Z | 1.7 rad/s | −1.7 rad/s (default) | 1.5 rad/s² | −1.5 rad/s² (default) |

Other stock settings: `cmd_vel_timeout = 0.5 s`, `publish_rate = 50 Hz`, `base_frame_id = base_footprint`, `use_stamped_vel = true`, `enable_odom_tf = true`, `publish_limited_velocity = true`.

---

## 8. Prerequisites

- Ubuntu **24.04** with **ROS 2 Jazzy** (desktop install)
- `colcon`, `rosdep`, `git`

ROS packages used by this workspace:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  ros-jazzy-rviz2 ros-jazzy-joy ros-jazzy-joy-teleop \
  ros-jazzy-turtlesim python3-numpy
```

---

## 9. Build

```bash
git clone https://github.com/DivyomSrivastava/Bumper_Bot.git bumperbot_ws
cd bumperbot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

> 💡 Always run `colcon build` from the **workspace root** (`bumperbot_ws/`), not from inside `src/`. Running it in `src/` creates stray `src/build`, `src/install` and `src/log` folders (these are already git-ignored).

---

## 10. Run

Every terminal needs `source install/setup.bash` first.

### 10.1 View the model in RViz

```bash
ros2 launch bumperbot_description display.launch.py
```

### 10.2 Simulate in Gazebo

```bash
ros2 launch bumperbot_description gazebo.launch.py
```

### 10.3 Start the controllers (second terminal)

```bash
# Custom Python controller
ros2 launch bumperbot_controller controller.launch.py use_python:=True

# Stock DiffDriveController
ros2 launch bumperbot_controller controller.launch.py use_simple_controller:=False
```

### 10.4 Drive the robot

**With a gamepad:**

```bash
ros2 launch bumperbot_controller joystick_teleop.launch.py
```

Hold the deadman button (button index 5), then use axis 1 for forward/back and axis 3 for turning.

**From the command line:**

```bash
ros2 topic pub -r 10 /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

### 10.5 Typical full session

| Terminal | Command |
| --- | --- |
| 1 | `ros2 launch bumperbot_description gazebo.launch.py` |
| 2 | `ros2 launch bumperbot_controller controller.launch.py use_python:=True` |
| 3 | `ros2 launch bumperbot_controller joystick_teleop.launch.py` |

---

## 11. ROS Interfaces Reference

| Name | Type | Direction | Provided by |
| --- | --- | --- | --- |
| `bumperbot_controller/cmd_vel` | `geometry_msgs/TwistStamped` | in | `joy_teleop`, CLI, or any node |
| `simple_velocity_controller/commands` | `std_msgs/Float64MultiArray` | out | `simple_controller` → `JointGroupVelocityController` |
| `/joint_states` | `sensor_msgs/JointState` | out | `joint_state_broadcaster` |
| `/robot_description` | `std_msgs/String` | out | `robot_state_publisher` |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | out | `robot_state_publisher`, `DiffDriveController` |
| `/joy` | `sensor_msgs/Joy` | out | `joy_node` |
| `add_two_ints` | `bumperbot_msgs/srv/AddTwoInts` | service | `simple_service_server` |
| `chatter` | `std_msgs/String` | pub/sub | `simple_publisher` / `simple_subscriber` |

---

## 12. Configuration Reference

### `controller.launch.py` arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `use_sim_time` | `True` | Use the Gazebo clock |
| `use_simple_controller` | `True` | `True` = custom velocity controller, `False` = stock `DiffDriveController` |
| `use_python` | `False` | When the custom controller is active, run `simple_controller.py` (the installed Python node) |
| `wheel_radius` | `0.033` | Wheel radius in metres |
| `wheel_separation` | `0.17` | Distance between wheels in metres |

### `joy_teleop.yaml`

| Mapping | Axis | Scale | Notes |
| --- | --- | --- | --- |
| `twist-linear-x` | 1 | 1.0 | Forward/back |
| `twist-angular-z` | 3 | 8.0 | Turning |
| Deadman | button 5 | n/a | Must be held to send commands |

Axis and button numbers depend on your controller. Check yours with `ros2 topic echo /joy`.

### `joy_config.yaml`

`device_id: 0`, `deadzone: 0.5`, `autorepeat_rate: 20.0`, `sticky_buttons: false`, `coalesce_interval_ms: 1`.

---

## 13. Practice Nodes (`bumperbot_py_examples`)

| Executable | Concept | Try it |
| --- | --- | --- |
| `simple_publisher` | Publisher with timer | `ros2 run bumperbot_py_examples simple_publisher` |
| `simple_subscriber` | Subscriber callback | `ros2 run bumperbot_py_examples simple_subscriber` |
| `simple_parameter` | Parameter declaration and validation (`simple_int_param` ≥ 0, `simple_string_param` ≤ 20 chars) | `ros2 run bumperbot_py_examples simple_parameter` |
| `simple_service_server` | Service server | `ros2 run bumperbot_py_examples simple_service_server` |
| `simple_service_client` | Async service client | `ros2 run bumperbot_py_examples simple_service_client 3 4` |
| `simple_tf_kinematics` | Static and dynamic TF broadcasting (`odom → bumperbot_base → bumperbot_top`) | `ros2 run bumperbot_py_examples simple_tf_kinematics` |
| `simple_turtlesim_kinematics` | Relative translation and rotation between two turtles | `ros2 run bumperbot_py_examples simple_turtlesim_kinematics` |

For the turtlesim example, spawn a second turtle first:

```bash
ros2 run turtlesim turtlesim_node
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

---

## 14. Git Workflow (`gitsync.sh`)

`gitsync.sh` stages everything, commits and pushes to the current branch in one step:

```bash
./gitsync.sh "added joystick teleop launch file"   # custom message
./gitsync.sh                                        # timestamped message
```

It prints `Nothing new to commit.` if the tree is clean, and then still pushes.

---

## 15. Troubleshooting

| Symptom | Likely cause and fix |
| --- | --- |
| `executable 'simple_controller' not found` | The default launch looks for a C++ executable that does not exist. Add `use_python:=True`. |
| `rosdep` fails on the key `sys` | `bumperbot_py_examples/package.xml` lists `<exec_depend>sys</exec_depend>`. `sys` is part of Python and is not a rosdep key; delete that line. |
| Robot is invisible or meshes missing in Gazebo | Source the workspace before launching so `GZ_SIM_RESOURCE_PATH` resolves to the install share folder. |
| Robot does not move in Gazebo | Controllers are not running. Start `controller.launch.py` in a second terminal and check `ros2 control list_controllers`. |
| Gamepad does nothing | Hold the deadman button (index 5) and verify axis and button numbers with `ros2 topic echo /joy`. |
| Odometry drifts in turns | Check that `wheel_separation` and `wheel_radius` match the model (see the warning in [section 6.2](#62-key-physical-parameters)). |
| Stray `src/build`, `src/install`, `src/log` | `colcon build` was run inside `src/`. Delete them and build from the workspace root. |
| VS Code is slow or the repo is huge | The IntelliSense cache (`.vscode/ipch`, `browse.vc.db*`) is git-ignored. Do not remove those lines from `.gitignore`. |

---

## 16. Credits and License

- Based on the **Bumperbot** project by **Antonio Brandi**. The `bumperbot_controller` package retains his Apache-2.0 attribution.
- Workspace assembled, extended and maintained by **[Divyom Srivastava](https://github.com/DivyomSrivastava)**.

Parts of the workspace are Apache-2.0 (`bumperbot_controller`). Other packages still contain `TODO` license declarations; add a `LICENSE` file before redistributing.

<div align="center">

⭐ If this workspace helped you, consider starring the repo.

</div>
