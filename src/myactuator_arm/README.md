# MyActuator Arm Package

ROS 2 package for controlling a 5 DOF robotic arm with MyActuator X6 motors via EtherCAT.

## Package Structure

```
myactuator_arm/
├── config/
│   ├── controllers/         # Controller configurations
│   ├── ethercat/           # Motor-specific EtherCAT configs with offsets
│   └── rviz/               # RViz visualization configs
├── description/
│   └── urdf/               # Robot description files
├── launch/                 # Launch files
└── scripts/                # Utility scripts
```

## Launch Files

### 1. Monitor Launch (Read-Only)

**Safest option** - Only reads motor positions and velocities. No control commands sent.

```bash
# Monitor all 5 joints (default)
ros2 launch myactuator_arm monitor.launch.py

# Monitor specific number of joints
ros2 launch myactuator_arm monitor.launch.py num_joints:=1  # Joint 1 only
ros2 launch myactuator_arm monitor.launch.py num_joints:=2  # Joints 1-2
ros2 launch myactuator_arm monitor.launch.py num_joints:=3  # Joints 1-3

# Echo joint states to terminal
ros2 topic echo /joint_states
```

**Features:**
- EtherCAT driver reads motor states only
- No controllers active (motors won't move)
- Safe for verifying communication and checking positions
- Use this first when adding new motors

### 2. Test Launch (No Virtual Wall)

For testing motors one by one during initial setup. **NO SAFETY LIMITS.**

```bash
# Test joint 1 only (EtherCAT slave position 0)
ros2 launch myactuator_arm test.launch.py num_joints:=1

# Test joints 1-2 (EtherCAT slave positions 0-1)
ros2 launch myactuator_arm test.launch.py num_joints:=2

# Test joints 1-3 (EtherCAT slave positions 0-2)
ros2 launch myactuator_arm test.launch.py num_joints:=3

# Test joints 1-4 (EtherCAT slave positions 0-3)
ros2 launch myactuator_arm test.launch.py num_joints:=4

# Test all 5 joints (EtherCAT slave positions 0-4)
ros2 launch myactuator_arm test.launch.py num_joints:=5
```

**WARNING:** No virtual wall protection. Motors can be moved freely. Use only for initial testing.

### 3. Free Mode Launch (With Virtual Wall)

For normal operation with safety limits enabled.

```bash
ros2 launch myactuator_arm free.launch.py
```

Uses virtual wall with spring-damper model at configured limits.

## Configuring Motors

### Step 0: Monitor Motor (Read-Only)

**Start here** - Verify EtherCAT communication without sending any commands:

```bash
ros2 launch myactuator_arm monitor.launch.py num_joints:=1
```

In another terminal, check position data:
```bash
ros2 topic echo /joint_states
```

Verify:
- Motor appears in /joint_states
- Position values are updating
- No errors in logs

### Step 1: Test Motor Movement

Once monitoring works, test with free movement (zero torque):

```bash
ros2 launch myactuator_arm test.launch.py num_joints:=1
```

Check the logs for:
- Motor enters "Operation Enabled" state
- Position feedback is being received
- Motor is in CST mode (mode 10)
- Motor can be moved freely by hand

### Step 2: Set Zero Position

Manually move the motor to the desired zero position, then run:

```bash
ros2 run myactuator_arm set_zero.py joint1
```

Copy the offset value to the motor config file:
- [config/ethercat/joint1_motor.yaml](config/ethercat/joint1_motor.yaml)
- In `tpdo` -> `channels` -> position line, update `offset: <value>`

### Step 3: Configure Limits

Edit the URDF file to set appropriate position limits:
- For testing: [description/urdf/arm_5dof.urdf.xacro](description/urdf/arm_5dof.urdf.xacro)
- For production with wall: Update [free.launch.py](launch/free.launch.py) virtual wall parameters

### Step 4: Rebuild

```bash
colcon build --packages-select myactuator_arm
source install/setup.bash
```

### Step 5: Repeat for Each Motor

Increment `num_joints` and repeat steps 0-4 for each additional motor.

**Example workflow when adding joint 3:**

```bash
# 1. Monitor only (safest first step) - will monitor all connected motors
ros2 launch myactuator_arm monitor.launch.py
ros2 topic echo /joint_states  # Verify joint3 appears

# Or monitor just up to joint 3 if you want
ros2 launch myactuator_arm monitor.launch.py num_joints:=3

# 2. Test with zero torque
ros2 launch myactuator_arm test.launch.py num_joints:=3

# 3. Set zero position
ros2 run myactuator_arm set_zero.py joint3

# 4. Update config/ethercat/joint3_motor.yaml with offset

# 5. Rebuild
colcon build --packages-select myactuator_arm
source install/setup.bash
```

## Motor Configuration Files

Each motor has its own EtherCAT configuration:

- `joint1_motor.yaml` - EtherCAT position 0, offset: -5.690862
- `joint2_motor.yaml` - EtherCAT position 1, offset: -2.654246
- `joint3_motor.yaml` - EtherCAT position 2, offset: -3.713241
- `joint4_motor.yaml` - EtherCAT position 3, offset: -2.833302
- `joint5_motor.yaml` - EtherCAT position 4, offset: 2.255961

## Utility Scripts

### set_zero.py

Calculate encoder offset to set current position as zero:

```bash
# Single joint
ros2 run myactuator_arm set_zero.py joint1

# All active joints
ros2 run myactuator_arm set_zero.py --all
```

### virtual_wall.py

Safety node that applies opposing torque at position limits. Automatically launched with [free.launch.py](launch/free.launch.py).

## EtherCAT Slave Mapping

| Joint Name | EtherCAT Position | Config File |
|------------|-------------------|-------------|
| joint1 | 0 | joint1_motor.yaml |
| joint2 | 1 | joint2_motor.yaml |
| joint3 | 2 | joint3_motor.yaml |
| joint4 | 3 | joint4_motor.yaml |
| joint5 | 4 | joint5_motor.yaml |

## Motor Specifications

- Model: MyActuator X6
- Protocol: CiA402 over EtherCAT
- Mode: CST (Cyclic Synchronous Torque, mode 10)
- Encoder: 131,072 counts/rev (17-bit)
- Position factor: 4.794e-5 rad/count
- Max torque: 20 Nm (configurable per joint)
- Max velocity: 16 rad/s

## Troubleshooting

**Motor not initializing:**
- Check EtherCAT connection: `ethercat slaves`
- Verify motor is at correct slave position
- Check motor is powered

**Position jumping/incorrect:**
- Verify offset in motor config file
- Rebuild after config changes
- Use `set_zero.py` to recalculate offset

**Motor hitting limits:**
- With test.launch.py: No protection, manually stop
- With free.launch.py: Virtual wall should engage
- Check limit configuration in URDF and launch file
