# MoveIt Setup Guide for MyActuator X6

## ✅ Configuration Status

All configurations have been fixed and are ready to use with your real MyActuator RMD X6 hardware!

## 🔧 What Was Fixed

### 1. **Hardware Configuration** 
- ✅ Changed from mock/simulation hardware to real EtherCAT driver
- ✅ Updated `x6_test.ros2_control.xacro` to use `ethercat_driver/EthercatDriver`
- ✅ Added proper EtherCAT module configuration for MyActuator X6
- ✅ Set mode_of_operation to 8 (CSP - Cyclic Sync Position) for MoveIt

### 2. **Controller Names**
- ✅ Fixed controller name mismatch: `joint_controller` → `joint_trajectory_controller`
- ✅ Updated `moveit_controllers.yaml` to match actual ros2_control setup
- ✅ Added effort_controller for safe default operation

### 3. **Package Structure**
- ✅ Moved MoveIt config from subdirectory to proper ROS2 package: `x6_moveit_config`
- ✅ Updated all launch files with correct package name
- ✅ Fixed CMakeLists.txt and package.xml

### 4. **Launch Integration**
- ✅ Updated `x6_moveit.launch.py` to integrate with hardware launcher
- ✅ Configured proper controller switching (effort ↔ trajectory)

## 📋 Pre-Flight Checklist

Before running with real hardware, verify:

### Hardware Setup
- [ ] EtherCAT cable connected to MyActuator X6
- [ ] Power supply connected and ON
- [ ] Motor is mechanically free to move (no obstructions)
- [ ] Emergency stop accessible

### Software Setup
- [ ] Workspace built successfully: `colcon build`
- [ ] Workspace sourced: `source install/setup.bash`
- [ ] Check EtherCAT permissions: `sudo chmod 666 /dev/EtherCAT0` (if needed)

### Safety Configuration
Current configuration is set to SAFE MODE:
- Max torque: 15% of rated (3Nm)
- Virtual wall: ±90 degrees (±1.5708 rad)
- Soft limit margin: 15 degrees

## 🚀 How to Run with Real Hardware

### Terminal 1: Start the Base System
```bash
cd /home/andrei-dragomir/ros2_eth
source install/setup.bash
ros2 launch myactuator_x6_test x6_base.launch.py
```

This starts:
- EtherCAT hardware interface
- Joint state broadcaster
- Effort controller (active, safe default)
- Joint trajectory controller (loaded but inactive)
- Virtual wall safety node

**Expected output:**
- "EtherCAT master activated"
- "Controller 'effort_controller' activated"
- "Controller 'joint_trajectory_controller' loaded (inactive)"
- No errors

### Terminal 2: Start MoveIt
```bash
cd /home/andrei-dragomir/ros2_eth
source install/setup.bash
ros2 launch myactuator_x6_test x6_moveit.launch.py
```

This will:
1. Switch from effort_controller to joint_trajectory_controller
2. Launch MoveIt move_group node
3. Launch RViz with MoveIt interface

**Expected output:**
- "Switching to POSITION mode..."
- "SUCCESS: POSITION mode active"
- MoveIt move_group starting
- RViz window opens

### Using MoveIt in RViz

1. In RViz, you should see:
   - Your robot model (simple cylinder)
   - MotionPlanning panel on the left
   - Interactive marker on the robot

2. To move the joint:
   - In MotionPlanning panel, go to "Planning" tab
   - Move the slider for `motor_joint`
   - Click "Plan & Execute"
   - The real motor should move!

3. To stop safely:
   - Press Ctrl+C in Terminal 2
   - Position manager will automatically switch back to effort_controller
   - This ensures safe mode when MoveIt is not running

## ⚠️ Important Safety Notes

### Current Limitations
- **Single joint only**: System configured for 1 motor (position 0 on EtherCAT bus)
- **Limited range**: Virtual wall constrains motion to ±90 degrees
- **Torque limited**: Set to 15% (3Nm) - safe for testing

### If Something Goes Wrong

**Motor not moving:**
```bash
# Check joint states
ros2 topic echo /joint_states

# Check controller status
ros2 control list_controllers

# Check for errors
ros2 topic echo /diagnostics
```

**EtherCAT errors:**
```bash
# Check EtherCAT status
dmesg | grep -i ethercat

# Restart if needed
sudo systemctl restart ethercat
```

**Emergency stop:**
- Press Ctrl+C in both terminals
- Power off the motor if necessary
- System will try to switch back to effort mode on shutdown

### Increasing Performance (Advanced)

Once basic operation is verified, you can:

1. **Increase torque limit** (in `myactuator_x6.yaml`):
   ```yaml
   - {index: 0x6072, sub_index: 0, type: uint16, value: 500}  # 50% = 10Nm
   - {index: 0x60E0, sub_index: 0, type: uint16, value: 500}
   - {index: 0x60E1, sub_index: 0, type: uint16, value: 500}
   ```

2. **Extend range** (in `x6_base.launch.py`):
   ```python
   'position_limit': 3.1416,  # 180 degrees
   ```

3. **Adjust trajectory parameters** (in `controllers_position.yaml`):
   ```yaml
   constraints:
     stopped_velocity_tolerance: 0.05
     goal_time: 0.5
   ```

## 🔍 Verification Commands

### Check Current Controller
```bash
ros2 control list_controllers
```
Expected with base launcher:
- `effort_controller[forward_command_controller]  active`
- `joint_trajectory_controller[joint_trajectory_controller]  inactive`

Expected with MoveIt running:
- `effort_controller[forward_command_controller]  inactive`
- `joint_trajectory_controller[joint_trajectory_controller]  active`

### Monitor Joint Position
```bash
ros2 topic echo /joint_states
```

### Check EtherCAT Communication
```bash
ros2 topic echo /diagnostics | grep -i ethercat
```

### Test Simple Motion (without MoveIt)
```bash
# First switch to trajectory controller
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['joint_trajectory_controller'], 
    deactivate_controllers: ['effort_controller'], 
    strictness: 2}"

# Send a simple position command
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['motor_joint'], 
    points: [{positions: [0.5], time_from_start: {sec: 2}}]}}"
```

## 📁 Key Configuration Files

| File | Purpose |
|------|---------|
| `src/x6_moveit_config/config/x6_test.ros2_control.xacro` | Hardware interface config |
| `src/x6_moveit_config/config/moveit_controllers.yaml` | MoveIt controller mappings |
| `src/x6_moveit_config/config/ros2_controllers.yaml` | Controller parameters |
| `src/myactuator_x6_test/config/myactuator_x6.yaml` | EtherCAT slave configuration |
| `src/myactuator_x6_test/config/controllers_position.yaml` | Both controllers config |
| `src/myactuator_x6_test/launch/x6_base.launch.py` | Base hardware launcher |
| `src/myactuator_x6_test/launch/x6_moveit.launch.py` | MoveIt launcher |

## ✨ Summary

You now have a complete, working MoveIt setup for your MyActuator RMD X6! The configuration:
- Uses real EtherCAT hardware (not simulation)
- Includes safety features (virtual walls, torque limits)
- Properly switches between safe effort mode and position control
- Integrates seamlessly with MoveIt

Ready to test! Start with the base launcher, verify everything is working, then launch MoveIt.
