# MoveIt Configuration for 5-DOF Arm - Real Hardware Setup

## What Was Done

### 1. Collision Geometry Integration ✅
- Copied all STL meshes from `robot_arm/assets/` to `description/meshes/`
- Added complete collision geometry to all 6 links in [arm_full.urdf.xacro](src/myactuator_arm/description/urdf/arm_full.urdf.xacro)
- Updated all mesh paths to use `package://myactuator_arm/description/meshes/`

### 2. MoveIt Configuration Package ✅
- Created `myactuator_arm_moveit_config` package at src level
- Generated complete MoveIt configuration using MoveIt Setup Assistant
- Planning group "arm" with all 5 joints (joint1-joint5)
- Collision matrix properly configured
- Group states: home, suppine, ready

### 3. Configuration Updates for Real Hardware ✅

#### Joint Limits ([joint_limits.yaml](src/myactuator_arm_moveit_config/config/joint_limits.yaml))
- **Velocity limits**: 16.0 rad/s (matches motor specs)
- **Acceleration limits**:
  - Joints 1-3 (X6 motors): 10.0 rad/s²
  - Joints 4-5 (X4 motors): 15.0 rad/s²
- **Scaling factors**: 10% for safe initial testing

#### Kinematics ([kinematics.yaml](src/myactuator_arm_moveit_config/config/kinematics.yaml))
- Solver timeout: 50ms (increased from 5ms)
- Retry attempts: 3
- Search resolution: 0.005

#### Controller Configuration ([ros2_controllers.yaml](src/myactuator_arm_moveit_config/config/ros2_controllers.yaml))
- Update rate: 100 Hz (appropriate for MoveIt)
- Position command interface
- Trajectory tolerances:
  - During execution: 0.05 rad
  - At goal: 0.01 rad
  - Goal time: 0.5 s
- Allows nonzero velocity at trajectory end (important for backdrivable robots)

#### Controller Names
- Updated from `arm_controller` → `joint_trajectory_controller`
- Matches existing arm controller configuration

### 4. Real Hardware Control Files ✅
Created files for real hardware integration:
- [myactuator_arm_real.ros2_control.xacro](src/myactuator_arm_moveit_config/config/myactuator_arm_real.ros2_control.xacro)
  - EtherCAT driver configuration
  - **CSP mode (mode 8)** for all joints - required for MoveIt
  - Control frequency: 100 Hz
- [myactuator_arm_real.urdf.xacro](src/myactuator_arm_moveit_config/config/myactuator_arm_real.urdf.xacro)
  - References arm_full.urdf.xacro for mechanical structure
  - Uses real hardware ros2_control configuration

## Important Notes

### Motor Operation Modes

The MyActuator motors support different CiA402 modes:
- **Mode 10 (CST - Cyclic Sync Torque)**: Used for effort control, backdrivable
- **Mode 8 (CSP - Cyclic Sync Position)**: Used for position control, **required for MoveIt**

**CRITICAL**: The `arm_full.urdf.xacro` has mode_of_operation=10 embedded in its ros2_control section. For MoveIt with real hardware, you need CSP mode (mode 8).

### Two Approaches for Real Hardware

#### Option A: Demo Mode (Simulation)
Uses mock hardware - good for testing MoveIt planning without motors:
```bash
ros2 launch myactuator_arm_moveit_config demo.launch.py
```

#### Option B: Real Hardware (RECOMMENDED APPROACH)

**Important**: Due to the embedded ros2_control section in arm_full.urdf.xacro being in CST mode (mode 10), you need to either:

1. **Modify arm_full.urdf.xacro temporarily** - Change mode_of_operation from 10 to 8:
   ```bash
   # Edit src/myactuator_arm/description/urdf/arm_full.urdf.xacro
   # Lines 432, 445, 458, 471, 484 (for each joint)
   # Change: <param name="mode_of_operation">10</param>
   # To:     <param name="mode_of_operation">8</param>
   
   # Rebuild
   colcon build --packages-select myactuator_arm
   source install/setup.bash
   ```

2. **OR use the existing launch with controller switching** (safer):
   ```bash
   # Terminal 1: Start with effort controller (CST mode)
   ros2 launch myactuator_arm arm_test.launch.py num_joints:=5
   
   # Terminal 2: Switch to position mode and launch MoveIt
   # (Will need a custom launch file to handle mode switching)
   ```

3. **OR create a parameterized URDF** (best long-term solution):
   - Modify arm_full.urdf.xacro to accept mode_of_operation as a xacro argument
   - Default to mode 10 for normal operation
   - Override to mode 8 when launching MoveIt

## Recommended Workflow for Real Hardware

### Step 1: Verify Motors are Calibrated
```bash
# Check encoder offsets in motor config files
cat src/myactuator_arm/config/ethercat/joint{1..5}_motor.yaml | grep offset
```

### Step 2: Test Individual Joints
```bash
# Start in monitor mode (read-only)
ros2 launch myactuator_arm arm_monitor.launch.py num_joints:=5

# Check joint states
ros2 topic echo /joint_states
```

### Step 3: Test Hardware with CSP Mode

Temporarily modify mode_of_operation to 8 in arm_full.urdf.xacro, then:

```bash
# Terminal 1: Start hardware (now in CSP mode)
ros2 launch robot_state_publisher robot_state_publisher.launch.py \
  robot_description:=$(xacro src/myactuator_arm/description/urdf/arm_full.urdf.xacro pkg_share:=$(ros2 pkg prefix myactuator_arm)/share/myactuator_arm)

# Start controller manager and load controllers
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller joint_trajectory_controller
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state joint_trajectory_controller active

# Terminal 2: Launch MoveIt
ros2 launch myactuator_arm_moveit_config move_group.launch.py

# Terminal 3: Launch RViz
ros2 launch myactuator_arm_moveit_config moveit_rviz.launch.py
```

### Step 4: Plan and Execute in RViz

1. In RViz, use the MotionPlanning panel
2. Drag the interactive marker to set a goal pose
3. Click "Plan" to generate trajectory
4. **IMPORTANT**: Start with small, slow movements
5. Click "Execute" to run on real hardware
6. Monitor for any issues

## Safety Checklist

Before running with real hardware:

- [ ] Motors are mechanically free to move
- [ ] Joint limits in URDF match mechanical limits
- [ ] Encoder offsets are calibrated for all joints
- [ ] Emergency stop is accessible
- [ ] Torque limits are appropriate (20 Nm for X6, 10 Nm for X4)
- [ ] Start with 10% velocity/acceleration scaling (default in joint_limits.yaml)
- [ ] Virtual wall safety is active (if using effort mode)
- [ ] Test each joint individually before full arm motion
- [ ] Keep hands clear during execution

## Troubleshooting

### MoveIt cannot plan
- Check if collision is disabled for adjacent links in SRDF
- Verify joint limits allow desired motion
- Increase planning time in RViz

### Robot doesn't move
- Check controller is active: `ros2 control list_controllers`
- Verify motor mode is CSP (mode 8): Check EtherCAT driver logs
- Check EtherCAT communication: `ros2 topic echo /diagnostics`

### Trajectory execution fails
- Check goal tolerances in ros2_controllers.yaml
- Verify goal_time is reasonable (currently 0.5s)
- Monitor joint states during execution

### Position errors/oscillation
- Motors are in wrong mode (should be CSP mode 8)
- PID gains need tuning (currently all zeros - using motor internal control)
- Check encoder offsets are correct

## Next Steps

1. **Create a proper launch file** for real hardware MoveIt operation that:
   - Loads URDF with CSP mode
   - Starts controller_manager with EtherCAT driver
   - Loads and activates joint_trajectory_controller
   - Launches move_group
   - Launches RViz

2. **Add controller switching support** (like x6_test package):
   - Start in effort mode (CST) for safety
   - Switch to position mode (CSP) when MoveIt launches
   - Switch back to effort mode on shutdown

3. **Tune trajectory parameters** based on actual performance:
   - Adjust acceleration limits
   - Fine-tune tolerances
   - Test scaling factors

## File Summary

### Modified Files
- `src/myactuator_arm/description/urdf/arm_full.urdf.xacro` - Added collision geometry, updated mesh paths
- `src/myactuator_arm/CMakeLists.txt` - Installs meshes folder
- `src/myactuator_arm_moveit_config/config/joint_limits.yaml` - Added acceleration limits
- `src/myactuator_arm_moveit_config/config/kinematics.yaml` - Fixed solver timeout
- `src/myactuator_arm_moveit_config/config/ros2_controllers.yaml` - Added trajectory tolerances
- `src/myactuator_arm_moveit_config/config/moveit_controllers.yaml` - Fixed controller name

### Created Files
- `src/myactuator_arm/description/meshes/` - All STL files (18 files)
- `src/myactuator_arm_moveit_config/config/myactuator_arm_real.ros2_control.xacro` - Real hardware config with CSP mode
- `src/myactuator_arm_moveit_config/config/myactuator_arm_real.urdf.xacro` - URDF for real hardware

## Build Commands

```bash
# Build both packages
colcon build --packages-select myactuator_arm myactuator_arm_moveit_config

# Source workspace
source install/setup.bash
```

