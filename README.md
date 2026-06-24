# open_manipulator_x_torque_control

ROS 2 Humble package for torque/effort control of the **OpenManipulator-X** (4-DOF serial robot arm by ROBOTIS). Supports Gazebo Fortress simulation and real hardware via Dynamixel SDK. Developed for the *Control de Sistemas No Lineales* laboratory course at UTEC.

Implemented controllers: **Gravity Compensation**, **Feedback Linearization** (joint-space and Cartesian), **Trajectory Optimization + TV-LQR**, and **Sliding Mode Control** (joint-space and Cartesian).

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
   - [Eigen 3](#1-eigen-3)
   - [Pinocchio](#2-pinocchio)
   - [Dynamixel SDK C++](#3-dynamixel-sdk-c)
   - [ROS 2 Humble](#4-ros-2-humble)
   - [Gazebo Fortress](#5-gazebo-fortress)
   - [OpenManipulator-X ROS Packages](#6-openmanipulator-x-ros-packages)
3. [Workspace Setup](#workspace-setup)
4. [Package Structure](#package-structure)
5. [Usage](#usage)
   - [Building the Package](#building-the-package)
   - [Gazebo Simulation Launch](#gazebo-simulation-launch)
   - [sim\_init\_config.yaml Reference](#sim_init_configyaml-reference)
   - [URDF and Xacro Files](#urdf-and-xacro-files)
   - [Forward Kinematics](#forward-kinematics)
   - [Feedback Linearization](#feedback-linearization)
   - [Trajectory Optimization and TV-LQR](#trajectory-optimization-and-tv-lqr)
   - [Sliding Mode Control](#sliding-mode-control)
6. [Real Hardware — USB Latency](#real-hardware--usb-latency)
7. [Data and Plots](#data-and-plots)

---

## Prerequisites

- Ubuntu 22.04 (Jammy)
- Python 3.10
- MATLAB (for trajectory generation and post-processing scripts)

---

## Installation

### 1. Eigen 3

```bash
sudo apt install -y libeigen3-dev
```

---

### 2. Pinocchio

Pinocchio is used to compute the mass matrix `M(q)`, the nonlinear terms `b(q, dq)` (Coriolis, centrifugal, gravity), the forward kinematics, and the Jacobian `J(q)` directly from the URDF.

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y git build-essential cmake curl lsb-release pkg-config
sudo apt install -qqy lsb-release curl

sudo mkdir -p /etc/apt/keyrings
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | sudo tee /etc/apt/keyrings/robotpkg.asc

echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] \
http://robotpkg.openrobots.org/packages/debian/pub \
$(lsb_release -cs) robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list

sudo apt update
sudo apt install -y robotpkg-py310-pinocchio

echo 'export PATH=/opt/openrobots/bin:$PATH' >> ~/.bashrc
echo 'export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> ~/.bashrc
echo 'export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH' >> ~/.bashrc

source ~/.bashrc
```

---

### 3. Dynamixel SDK C++

Required for communication with the XM430-W350 servo motors on the real robot.

```bash
cd ~/Documents
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/c++

cmake -B cmake_build
cmake --build cmake_build
sudo cmake --install cmake_build
sudo ldconfig

# Add your user to the dialout group for USB access
sudo usermod -a -G dialout $USER
# Log out and back in for this to take effect
```

---

### 4. ROS 2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt source
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS 2 Humble Desktop + dev tools
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

# Source ROS 2 automatically
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

### 5. Gazebo Fortress

Gazebo Fortress (Ignition Fortress) is the simulation environment used for all controllers. The `ros_gz` bridge connects Gazebo topics to ROS 2.

```bash
sudo apt-get update
sudo apt-get install lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install ignition-fortress

# ROS 2 ↔ Gazebo bridge and effort controllers
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-gz-ros2-control \
  ros-humble-gz-ros2-control-demos \
  ros-humble-effort-controllers
```

---

### 6. OpenManipulator-X ROS Packages

Additional ROS 2 control packages required by the workspace:

```bash
sudo apt install \
  ros-humble-ros2-control \
  ros-humble-moveit* \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-position-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-gripper-controllers \
  ros-humble-hardware-interface \
  ros-humble-xacro
```

---

## Workspace Setup

```bash
mkdir -p ~/open_manx_ws/src
cd ~/open_manx_ws/src

git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/open_manipulator.git
git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
git clone https://github.com/smorales2405/open_manipulator_x_torque_control

cd ~/open_manx_ws
colcon build --symlink-install

echo 'source ~/open_manx_ws/install/local_setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

## Package Structure

```
open_manipulator_x_torque_control/
├── config/
│   ├── sim_init_config.yaml          # Gazebo initial pose, obstacle, and dynamics scales
│   ├── effort_controllers.yaml       # ros2_control effort controller config
│   ├── motor_params.yaml             # Dynamixel motor parameters for real HW
│   ├── motorXM430W350T_params.yaml   # Identified torque→current model (assembled final)
│   ├── hw_sinusoidal_torque_params.yaml  # Sinusoidal excitation parameters
│   ├── hw_friction_sweep_params.yaml     # Constant-velocity friction sweep parameters
│   ├── fkin_params.yaml              # Forward kinematics display parameters
│   └── fkin_rviz.rviz                # RViz config for FK visualization
├── launch/
│   ├── torque_sim.launch.py          # Main Gazebo simulation launcher
│   ├── gravity_comp_gz.launch.py     # Gravity compensation in simulation
│   ├── fkin_rviz.launch.py           # Forward kinematics + RViz display
│   └── friction_sweep.launch.py      # Constant-velocity friction sweep on real HW
│   # (HW gravity comp y TV-LQR se corren con `ros2 run`; auto-cargan motorXM430W350T_params.yaml)
├── src/
│   ├── Feedback Linearization/
│   │   ├── instructor/               # Complete implementations (reference)
│   │   ├── student/                  # Templates for students to complete
│   │   └── MATLAB/                   # Post-processing and plotting scripts
│   ├── Sliding Mode/
│   │   ├── instructor/               # Complete implementations (reference)
│   │   ├── student/                  # Templates for students to complete
│   │   └── MATLAB/                   # Post-processing and plotting scripts
│   ├── Trajectory Optimization TV-LQR/
│   │   ├── instructor/               # Complete ROS 2 nodes (no modification needed)
│   │   ├── references/               # Exported trajectory files from MATLAB
│   │   └── MATLAB/                   # Trajectory generation and analysis scripts
│   ├── Gravity Compensation/         # Gravity compensation nodes (sim and HW)
│   ├── Forward Kinematics/           # HW FK node and RViz display node
│   ├── Identification/               # Torque→current model ID (excitation node + Python OLS/friction)
│   └── Diagnostics/                  # Low-level torque diagnostic nodes
├── urdf/
│   └── open_manipulator_x.urdf       # Robot URDF (used by Pinocchio at runtime)
├── xacro/
│   ├── open_manipulator_x.urdf.xacro            # Main robot Xacro (used by Gazebo as plant)
│   ├── open_manipulator_x_effort_robot.urdf.xacro  # Xacro with effort controllers + simulation parameters
│   └── open_manipulator_x_effort.ros2_control.xacro # ros2_control hardware interface definition
├── worlds/
│   └── empty_world_fortress.sdf      # Empty world for Gazebo Fortress
├── data/
│   ├── lab4/  sim/  real/            # Feedback Linearization experiment data
│   ├── lab5/  sim/  real/            # TV-LQR experiment data
│   ├── lab6/  sim/                   # Sliding Mode experiment data
│   ├── gravity_comp/                 # Gravity compensation data
│   ├── identification/               # Motor identification data
│   └── diagnostics/                  # Diagnostic torque data
└── scripts/
    ├── kill_gazebo.sh                # Force-kill all Gazebo processes
    └── xacro_oneline.sh              # Generate URDF from Xacro (one-liner)
```

---

## Usage

### Building the Package

After modifying any C++ source file, recompile only this package from the workspace root:

```bash
cd ~/open_manx_ws
colcon build --symlink-install \
    --packages-select open_manipulator_x_torque_control
source install/setup.bash
```

---

### Gazebo Simulation Launch

All simulation-based controllers use the same Gazebo launch file. Open **Terminal 1** and run:

```bash
cd ~/open_manx_ws
source install/setup.bash
ros2 launch open_manipulator_x_torque_control torque_sim.launch.py
```

This launches Gazebo Fortress with `ros2_control` effort controllers. The robot's initial pose and plant dynamics are read from `config/sim_init_config.yaml`.

---

### sim\_init\_config.yaml Reference

Located at `config/sim_init_config.yaml`. Edit this file **before** launching Gazebo to configure the simulation.

```yaml
# Initial pose
use_fixed_init: true          # false = start at gravity equilibrium
q_init: [0.0, -0.5, 0.3, 0.785]  # [q1, q2, q3, q4] in radians

# Obstacle (MDF arch, used in TV-LQR Lab 5 Activity 2)
spawn_obstacle: false
obstacle_pose: [0.1125, 0.0, 0.0, 0.0, 0.0, 1.5708]  # [x,y,z,r,p,yaw]

# Plant dynamics scales (used in Sliding Mode Lab 6 robustness tests)
mass_inertia_scale: 1.0   # 1.2 = +20% mass/inertia perturbation
damping_scale:      1.0   # 1.1 = +10% viscous damping perturbation
friction_scale:     0.0   # 0.0 = no friction in simulation

# Additional end-effector load (used in Sliding Mode Lab 6 load tests)
spawn_load:    false
load_radius:   0.03    # [m]
load_height:   0.04    # [m]
load_mass:     0.100   # [kg]
load_x_offset: 0.015   # [m]
```

**Common configurations by experiment:**

| Experiment | `use_fixed_init` | `q_init` | Notes |
|---|---|---|---|
| Feedback Linearization (Lab 4) | `true` | `[0,0,0,0]` or free | Nominal plant |
| TV-LQR Act. 1 (Lab 5) | `true` | `[1.5708, 0.0, 0.5236, 1.0472]` | `spawn_obstacle: false` |
| TV-LQR Act. 2 (Lab 5) | `true` | `[1.5708, 0.0, 0.5236, 1.0472]` | `spawn_obstacle: true` |
| SMC Articular (Lab 6) | `true` | `[0.0, -0.5, 0.3, 0.785]` | Nominal: `mass_inertia_scale: 1.0` |
| SMC Articular — incertidumbre | `true` | `[0.0, -0.5, 0.3, 0.785]` | `mass_inertia_scale: 1.2`, `damping_scale: 1.1` |
| SMC Articular — carga | `true` | `[0.0, -0.5, 0.3, 0.785]` | `spawn_load: true` |
| SMC Cartesiano (Lab 6) | `false` | — | Robot starts at gravity equilibrium |

---

### URDF and Xacro Files

| File | Purpose |
|---|---|
| `urdf/open_manipulator_x.urdf` | Static URDF loaded by Pinocchio at runtime for dynamics and kinematics computations. All C++ nodes that use Pinocchio reference this file via `PACKAGE_URDF_DIR`. |
| `xacro/open_manipulator_x.urdf.xacro` | Main robot description used as the **nominal model** by Gazebo. Modified by `mass_inertia_scale`, `damping_scale`, and `friction_scale` from `sim_init_config.yaml` to simulate parametric uncertainty. |
| `xacro/open_manipulator_x_effort_robot.urdf.xacro` | Top-level Xacro that includes the robot description and adds `ros2_control` effort interfaces. Also supports `spawn_load` to attach a cylindrical payload to the end-effector for load perturbation tests. |
| `xacro/open_manipulator_x_effort.ros2_control.xacro` | Defines the `ros2_control` hardware interface (effort joints) wired to the Gazebo effort plugin. |

To regenerate the URDF from the main Xacro (e.g., to update Pinocchio's model):

```bash
cd ~/open_manx_ws/src/open_manipulator_x_torque_control
bash scripts/xacro_oneline.sh
```

---

### Forward Kinematics

The `hw_fkin_node` connects to the real robot via USB, reads joint positions, and prints the joint angles and end-effector pose `[x, y, z, φ]` in real time. Useful for verifying the robot's initial configuration before running a trajectory.

**Terminal 2** (after verifying `/dev/ttyUSB0` is available):

```bash
ros2 run open_manipulator_x_torque_control hw_fkin_node
```

For RViz visualization of the forward kinematics:

```bash
ros2 launch open_manipulator_x_torque_control fkin_rviz.launch.py
```

This launches `hw_fkin_node` together with `fkin_display_node.py`, which publishes the end-effector pose as a marker, and opens RViz with the pre-configured `config/fkin_rviz.rviz` layout.

---

### Feedback Linearization

Implements computed-torque / feedback linearization control in joint space (**Activity 1**) and input-output linearization in Cartesian space (**Activity 2**). Uses Pinocchio for `M(q)` and `b(q, dq)`.

> **Student files** (templates to complete):
> `src/Feedback Linearization/student/gz_fl_control_node_base.cpp`,
> `gz_io_control_node_base.cpp`, `hw_fl_control_node_base.cpp`, `hw_io_control_node_base.cpp`

#### Activity 1 — Joint-space FL

Launch Gazebo first (Terminal 1), then in **Terminal 2**:

```bash
# Simulation
ros2 run open_manipulator_x_torque_control gz_fl_control_node \
    --ros-args -p test_num:=1 -p t_sim:=20.0

# Real hardware
ros2 run open_manipulator_x_torque_control hw_fl_control_node \
    --ros-args -p log_id:=1 -p t_imp:=20.0
```

| Parameter | Description |
|---|---|
| `test_num` / `log_id` | Numeric ID appended to the output CSV filename |
| `t_sim` / `t_imp` | Experiment duration in seconds |

Data saved to: `data/lab4/sim/act1/gz_fl_data_<N>.csv` and `data/lab4/real/act1/hw_fl_data_<N>.csv`

**MATLAB post-processing:** run `src/Feedback Linearization/MATLAB/plots_FL_control.m`

```matlab
mode     = 'sim';   % 'sim' or 'real'
test_num = 1;
EXPORT_FIGS = false; % true to save figures to plots/
```

#### Activity 2 — Cartesian IO Linearization

```bash
# Simulation
ros2 run open_manipulator_x_torque_control gz_io_control_node \
    --ros-args -p test_num:=1 -p t_sim:=20.0

# Real hardware
ros2 run open_manipulator_x_torque_control hw_io_control_node \
    --ros-args -p log_id:=1 -p t_imp:=20.0
```

Data saved to: `data/lab4/sim/act2/` and `data/lab4/real/act2/`

**MATLAB post-processing:** run `src/Feedback Linearization/MATLAB/plots_IO_control.m`

```matlab
mode     = 'sim';   % 'sim' or 'real'
test_num = 1;
EXPORT_FIGS = false;
```

---

### Trajectory Optimization and TV-LQR

Trajectory generation is done entirely in MATLAB using direct collocation (`fmincon`) and a time-varying LQR gain computed by solving the Riccati equation backwards. The resulting trajectory and gains are exported as text files and consumed by the ROS 2 nodes.

> **Student MATLAB files** (templates to complete):
> `src/Trajectory Optimization TV-LQR/MATLAB/student/lab5_act1_student.m` (Activity 1),
> `src/Trajectory Optimization TV-LQR/MATLAB/student/lab5_act2_student.m` (Activity 2 — with MDF obstacle)

#### Workflow

**Step 1 — Generate trajectory in MATLAB**

Complete and run `lab5_act1_student.m` (or `lab5_act2_student.m`). Then export references to `references/`:

```matlab
% After lab5_act1_student.m leaves N, Ts, nx, nu, x0, yf, Xref, Uref, K_TV in workspace:
run('src/Trajectory Optimization TV-LQR/MATLAB/Lab5_Export_Refs.m')
```

This writes: `time_ref.txt`, `q_ref.txt`, `dq_ref.txt`, `u_ref.txt`, `K_TV.txt`, `tau_gravity.txt` to the `references/` folder.

**Step 2 — Configure `sim_init_config.yaml`**

Set `q_init` to the trajectory's initial joint state `x0[0:4]` (in radians):

```yaml
use_fixed_init: true
q_init: [1.5708, 0.0, 0.5236, 1.0472]   # Activity 1 initial state
spawn_obstacle: false                     # true for Activity 2
```

**Step 3 — Launch Gazebo simulation** (Terminal 1)

```bash
ros2 launch open_manipulator_x_torque_control torque_sim.launch.py
```

**Step 4 — Run TV-LQR node** (Terminal 2)

```bash
# Simulation (t_sim = tf = N * Ts)
ros2 run open_manipulator_x_torque_control gz_tvlqr_node \
    --ros-args -p test_num:=1 -p t_sim:=1.5

# Real hardware (t_run = tf = N * Ts)
ros2 run open_manipulator_x_torque_control hw_tvlqr_node \
    --ros-args -p test_num:=1 -p t_run:=1.5
```

| Parameter | Description |
|---|---|
| `test_num` | ID appended to output CSV; use different IDs for Act. 1 and Act. 2 |
| `t_sim` / `t_run` | Must equal `N * Ts` from MATLAB |
| `torque_scale` | (optional) Scale factor on applied torque, useful for a progressive HW ramp-up |

Data saved to: `data/lab5/sim/data_log_sim_lab5_<N>.csv` and `data/lab5/real/data_log_real_lab5_<N>.csv`

**Step 5 — MATLAB analysis**

```matlab
% Individual run (sim or real):
mode     = 'sim';   % 'sim' or 'real'
test_num = 1;
run('src/Trajectory Optimization TV-LQR/MATLAB/Lab5_Plot_SimReal.m')

% Side-by-side comparison:
test_num_sim  = 1;
test_num_real = 1;
run('src/Trajectory Optimization TV-LQR/MATLAB/Lab5_Compare_Sim_Real.m')
```

---

### Sliding Mode Control

Implements SMC in joint space (**Activity 1**) and Cartesian space (**Activity 2**), both in Gazebo simulation only. The Xacro files support parametric uncertainty and end-effector load perturbation via `sim_init_config.yaml`.

> **Student files** (templates to complete):
> `src/Sliding Mode/student/gz_SMC_joint_node_base.cpp` (Activity 1),
> `src/Sliding Mode/student/gz_SMC_cart_node_base.cpp` (Activity 2)

#### Activity 1 — Joint-space SMC

Launch Gazebo first (Terminal 1), then in **Terminal 2**:

```bash
# With sign(s) switching function
ros2 run open_manipulator_x_torque_control gz_smc_joint_node \
    --ros-args -p rho_func:=sign -p test_num:=1 -p t_sim:=20.0

# With sat(s/phi) switching function
ros2 run open_manipulator_x_torque_control gz_smc_joint_node \
    --ros-args -p rho_func:=sat -p phi:=0.05 -p test_num:=2 -p t_sim:=20.0
```

| Parameter | Description |
|---|---|
| `rho_func` | Switching function: `sign` or `sat` |
| `phi` | Boundary layer width (only used when `rho_func:=sat`) |
| `test_num` | ID appended to output CSV |
| `t_sim` | Simulation duration in seconds |

Data saved to: `data/lab6/sim/act1/gz_smc_joint_<rho>_<N>.csv`

**MATLAB post-processing:**

```matlab
% Individual run:
rho_func = 'sign';  % 'sign' or 'sat'
test_num = 1;
run('src/Sliding Mode/MATLAB/plots_SMC_joint.m')

% sign vs sat comparison:
test_num_sign = 1;
test_num_sat  = 2;
run('src/Sliding Mode/MATLAB/plots_SMC_comp_sf_joint.m')

% Robustness comparison (nominal vs uncertainty or load):
robustness_test = 'params';  % 'params' or 'load'
test_num_nominal = 2;
test_num_perturb = 3;
run('src/Sliding Mode/MATLAB/plots_SMC_comp_rob_joint.m')
```

#### Activity 2 — Cartesian SMC

The node performs a quintic transition of duration `T_trans = 3 s` before the SMC phase. `t_sim` refers only to the SMC phase; total runtime = `T_trans + t_sim`.

Set `use_fixed_init: false` in `sim_init_config.yaml` so the robot starts at gravity equilibrium.

```bash
# With sign(s)
ros2 run open_manipulator_x_torque_control gz_smc_cart_node \
    --ros-args -p rho_func:=sign -p test_num:=1 -p t_sim:=20.0

# With sat(s/phi)
ros2 run open_manipulator_x_torque_control gz_smc_cart_node \
    --ros-args -p rho_func:=sat -p phi:=0.25 -p test_num:=2 -p t_sim:=20.0
```

Data saved to: `data/lab6/sim/act2/gz_smc_cart_<rho>_<N>.csv`

**MATLAB post-processing:**

```matlab
% Individual run:
rho_func = 'sat';
test_num = 2;
run('src/Sliding Mode/MATLAB/plots_SMC_cart.m')

% sign vs sat comparison:
run('src/Sliding Mode/MATLAB/plots_SMC_comp_sf_cart.m')

% Robustness comparison:
robustness_test = 'load';  % 'params' or 'load'
run('src/Sliding Mode/MATLAB/plots_SMC_comp_rob_cart.m')
```

---

## Real Hardware — USB Latency

The OpenManipulator-X communicates via a USB-to-RS485 adapter. Reducing the USB port latency improves control loop timing significantly.

**Each time the robot is connected**, check the assigned port and reduce its latency:

```bash
# Verify the USB port
ls /dev/ttyUSB*

# Reduce latency timer to 1 ms (default is 16 ms)
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

If the robot is not at `/dev/ttyUSB0`, replace `ttyUSB0` with the correct port number.

> **Note:** make sure your user belongs to the `dialout` group (done during Dynamixel SDK installation). If not, run `sudo usermod -a -G dialout $USER` and log out/in.

---

## Data and Plots

All experiment data is saved as CSV files under `data/`. Each controller writes one CSV per run, identified by the `test_num` (simulation) or `log_id` (hardware) parameter.

| Folder | Contents |
|---|---|
| `data/lab4/sim/act1/` | Feedback Linearization — joint space, simulation |
| `data/lab4/sim/act2/` | IO Linearization — Cartesian, simulation |
| `data/lab4/real/act1/` | Feedback Linearization — joint space, real HW |
| `data/lab4/real/act2/` | IO Linearization — Cartesian, real HW |
| `data/lab5/sim/` | TV-LQR — simulation |
| `data/lab5/real/` | TV-LQR — real HW |
| `data/lab6/sim/act1/` | SMC joint space — simulation |
| `data/lab6/sim/act2/` | SMC Cartesian — simulation |
| `data/gravity_comp/` | Gravity compensation experiments |
| `data/identification/` | Motor dynamics identification |
| `data/diagnostics/` | Low-level sinusoidal torque diagnostics |

MATLAB plotting scripts are located alongside each controller's source in `src/<Controller>/MATLAB/`. Set `EXPORT_FIGS = true` inside the scripts to save figures automatically to `plots/`.
