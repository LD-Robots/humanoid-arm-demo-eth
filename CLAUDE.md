# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# Standard build
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Source workspace
source install/setup.bash

# EtherCAT permissions (if needed)
sudo chmod 666 /dev/EtherCAT0
```

## High-Level Architecture

### Core Components

- **Hardware Layer**: IgH EtherCAT Master + MyActuator X6 motors (CiA402 protocol)
- **Driver Layer**: `ethercat_driver_ros2` (ICube-Robotics git submodule) provides Hardware Interface plugin
- **Control Layer**: ROS2 Control framework with controller_manager
- **Application Layer**:
  - `myactuator_x6_test`: Single/dual motor testing with MoveIt2 integration
  - `myactuator_arm`: 5-DOF robotic arm control

### Critical Design Pattern: Controller Switching

This system uses **TWO controllers per configuration**:

1. **Effort Controller** (CST mode 10)
   - Safe default mode, back-drivable
   - Used during startup and shutdown
   - Active with virtual wall safety system
   - Motor can be moved by hand

2. **Trajectory Controller** (CSP mode 8)
   - Position control mode
   - Required for MoveIt2 integration
   - Not back-drivable during operation
   - Used for planned motion execution

**Switching Requirements:**
- Must deactivate current controller before activating new one
- Strictness parameter: 2 (STRICT) in all switch operations
- Managed automatically by `position_mode_manager.py` for MoveIt integration
- **NEVER switch controllers while executing trajectories**

### Safety System: Virtual Wall

- Spring-damper model applies opposing torque near position limits
- Configurable stiffness and damping parameters
- Engages before hard limits are reached
- Only active with effort controller (CST mode)
- Default: ±90 degrees, 15-degree soft margin

## Common Workflows

### Single Motor Testing (X6 Test Package)

```bash
# Terminal 1: Start base system (EtherCAT + effort controller + virtual wall)
ros2 launch myactuator_x6_test x6_base.launch.py

# Terminal 2: Start MoveIt (automatically switches to trajectory controller)
ros2 launch myactuator_x6_test x6_moveit.launch.py

# Monitor joint states
ros2 topic echo /joint_states

# Check controller status
ros2 control list_controllers
```

Expected controller states:
- With base launcher only: `effort_controller` active
- With MoveIt running: `joint_trajectory_controller` active

### 5-DOF Arm Development (Adding Motors Incrementally)

```bash
# Step 1: Monitor only (safest, read-only)
ros2 launch myactuator_arm arm_monitor.launch.py num_joints:=1
ros2 topic echo /joint_states  # Verify motor appears

# Step 2: Test motor with zero torque (free movement)
ros2 launch myactuator_arm arm_test.launch.py num_joints:=1

# Step 3: Calibrate zero position
# Manually move motor to desired zero position, then:
ros2 run myactuator_arm set_zero.py joint1
# Copy the offset value to config/ethercat/joint1_motor.yaml

# Step 4: Rebuild after config changes
colcon build --packages-select myactuator_arm
source install/setup.bash

# Step 5: Repeat for additional joints
ros2 launch myactuator_arm arm_monitor.launch.py num_joints:=2
# Continue calibration workflow for joint2...
```

## Critical Technical Details

### MyActuator X6 Specifications

- Model: MyActuator RMD X6-60
- Gear Ratio: 19.612:1
- Rated Torque: 20 Nm
- Peak Torque: 60 Nm (3x rated, short duration)
- Rated Speed: 153 RPM output (16 rad/s)
- Encoder: 131,072 counts/rev (17-bit output encoder, post-gearbox)
- Control Frequency: 100 Hz
- Communication: EtherCAT (CiA402 protocol)

### Unit Conversions

**Position:**
- `rad → counts`: factor = 20861.0
- `counts → rad`: factor = 4.794e-5

**Velocity:**
- `rad/s → counts/s`: factor = 20861.0
- `counts/s → rad/s`: factor = 4.794e-5

**Effort/Torque:**
- `Nm → permille of rated`: factor = 50.0
- `permille → Nm`: factor = 0.02
- Example: 10 Nm = 500 permille (50% of 20 Nm rated)

### CiA402 Operation Modes

- **Mode 8 (CSP)**: Cyclic Sync Position - Used with MoveIt2, not back-drivable
- **Mode 9 (CSV)**: Cyclic Sync Velocity - Not currently used
- **Mode 10 (CST)**: Cyclic Sync Torque - Default mode, back-drivable, safe

### EtherCAT Configuration

- Master ID: 0
- Slave positions: Mapped sequentially (0-4 for 5-DOF arm)
- Each motor: Separate config file with calibrated encoder offset
- Control cycle: 100 Hz (10ms period)

### CiA402 SDO Objects

Key objects used in configuration:
- `0x6040`: Controlword
- `0x6041`: Statusword
- `0x6060`: Mode of operation
- `0x6071`: Target torque
- `0x6072`: Max torque (default: 1000 = 100% = 20 Nm)
- `0x607A`: Target position
- `0x6064`: Position actual value
- `0x60FF`: Target velocity
- `0x606C`: Velocity actual value
- `0x6077`: Torque actual value
- `0x60E0`: Positive torque limit
- `0x60E1`: Negative torque limit

## Configuration File Locations

### Motor Configurations
- X6 Test: [src/myactuator_x6_test/config/myactuator_x6.yaml](src/myactuator_x6_test/config/myactuator_x6.yaml)
- Arm Joint 1: [src/myactuator_arm/config/ethercat/joint1_motor.yaml](src/myactuator_arm/config/ethercat/joint1_motor.yaml)
- Arm Joint 2-5: `joint{2-5}_motor.yaml` in same directory

### Controller Configurations
- X6 Test: [src/myactuator_x6_test/config/controllers_position.yaml](src/myactuator_x6_test/config/controllers_position.yaml)
- Arm: [src/myactuator_arm/config/controllers/arm_controllers.yaml](src/myactuator_arm/config/controllers/arm_controllers.yaml)

### Robot Descriptions (URDF)
- X6 Test: [src/myactuator_x6_test/urdf/x6_test.urdf.xacro](src/myactuator_x6_test/urdf/x6_test.urdf.xacro)
- Arm: [src/myactuator_arm/description/urdf/arm_full.urdf.xacro](src/myactuator_arm/description/urdf/arm_full.urdf.xacro)
- MoveIt: [src/x6_moveit_config/config/x6_test.urdf.xacro](src/x6_moveit_config/config/x6_test.urdf.xacro)

### Launch Files
- X6 Base: [src/myactuator_x6_test/launch/x6_base.launch.py](src/myactuator_x6_test/launch/x6_base.launch.py)
- X6 MoveIt: [src/myactuator_x6_test/launch/x6_moveit.launch.py](src/myactuator_x6_test/launch/x6_moveit.launch.py)
- Arm Monitor: [src/myactuator_arm/launch/arm_monitor.launch.py](src/myactuator_arm/launch/arm_monitor.launch.py)
- Arm Test: [src/myactuator_arm/launch/arm_test.launch.py](src/myactuator_arm/launch/arm_test.launch.py)

### Utility Scripts
- Virtual Wall: [src/myactuator_x6_test/scripts/virtual_wall.py](src/myactuator_x6_test/scripts/virtual_wall.py)
- Position Manager: [src/myactuator_x6_test/scripts/position_mode_manager.py](src/myactuator_x6_test/scripts/position_mode_manager.py)
- Set Zero: [src/myactuator_arm/scripts/set_zero.py](src/myactuator_arm/scripts/set_zero.py)

## Important Constraints

### Controller Switching
- Must deactivate current controller before activating new one
- Strictness parameter must be 2 (STRICT)
- Position mode manager handles this automatically for MoveIt
- Switching during trajectory execution will cause motion to abort

### Safety Configuration
- Default torque limit: 15% (3 Nm) for testing
- Can be increased up to 100% (20 Nm rated) or 300% (60 Nm peak)
- Virtual wall default: ±90 degrees with 15-degree soft margin
- Emergency stop: Ctrl+C in both terminals (automatically returns to effort mode)

### EtherCAT Requirements
- Requires IgH EtherCAT Master kernel module
- May need root permissions for device access
- EtherCAT bus must be activated before controller_manager starts
- Verify connection: `ethercat slaves` (may require sudo)

## Troubleshooting Commands

```bash
# Check controller status
ros2 control list_controllers

# Monitor joint states (position, velocity, effort)
ros2 topic echo /joint_states

# Check diagnostics
ros2 topic echo /diagnostics

# EtherCAT diagnostics (may need sudo)
dmesg | grep -i ethercat
ethercat slaves
ethercat master

# Manual controller switching (use with caution)
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['joint_trajectory_controller'],
    deactivate_controllers: ['effort_controller'],
    strictness: 2}"
```

## Common Issues

**Motor not initializing:**
- Check EtherCAT connection: `ethercat slaves`
- Verify motor at correct slave position
- Check motor power supply
- Review logs for state machine errors

**Position jumping/incorrect:**
- Verify encoder offset in motor config file
- Rebuild after config changes: `colcon build --packages-select <package>`
- Recalibrate using `set_zero.py`

**Controller switching fails:**
- Check that both controllers are defined in URDF
- Verify controller names match exactly
- Ensure no trajectory is executing during switch
- Check logs for controller_manager errors

**Motor hitting limits:**
- With `test.launch.py`: No protection, stop manually
- With `x6_base.launch.py` or `free.launch.py`: Virtual wall should engage
- Verify limit configuration in URDF matches launch file parameters

## Documentation References

For detailed information, see:
- [MOVEIT_SETUP_GUIDE.md](MOVEIT_SETUP_GUIDE.md) - Complete MoveIt setup and operation
- [src/myactuator_arm/README.md](src/myactuator_arm/README.md) - Arm configuration workflow
- [src/myactuator_x6_test/config/myactuator_x6_specs.md](src/myactuator_x6_test/config/myactuator_x6_specs.md) - Motor specifications
- [src/ethercat_driver_ros2/README.md](src/ethercat_driver_ros2/README.md) - Driver documentation

External dependencies:
- EtherCAT Driver: [ICube-Robotics/ethercat_driver_ros2](https://github.com/ICube-Robotics/ethercat_driver_ros2)
- IgH EtherCAT Master: [etherlab.org](https://etherlab.org/en/ethercat/)
