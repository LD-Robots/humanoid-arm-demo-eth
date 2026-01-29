#!/bin/bash
# MoveIt Setup Verification Script

echo "========================================="
echo "MoveIt Configuration Verification"
echo "========================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if workspace is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ ROS not sourced${NC}"
    echo "  Run: source /opt/ros/<distro>/setup.bash"
    exit 1
fi
echo -e "${GREEN}✓ ROS environment loaded (ROS_DISTRO=$ROS_DISTRO)${NC}"

# Check if workspace is built
if [ ! -f "install/setup.bash" ]; then
    echo -e "${RED}✗ Workspace not built${NC}"
    echo "  Run: colcon build"
    exit 1
fi
echo -e "${GREEN}✓ Workspace built${NC}"

# Check for required packages
PACKAGES=("x6_moveit_config" "myactuator_x6_test")
for pkg in "${PACKAGES[@]}"; do
    if [ -d "install/$pkg" ]; then
        echo -e "${GREEN}✓ Package '$pkg' installed${NC}"
    else
        echo -e "${RED}✗ Package '$pkg' not found${NC}"
        exit 1
    fi
done

# Check for key configuration files
echo ""
echo "Checking configuration files..."

FILES=(
    "src/x6_moveit_config/config/x6_test.ros2_control.xacro"
    "src/x6_moveit_config/config/moveit_controllers.yaml"
    "src/x6_moveit_config/config/ros2_controllers.yaml"
    "src/myactuator_x6_test/config/myactuator_x6.yaml"
    "src/myactuator_x6_test/config/controllers_position.yaml"
    "src/myactuator_x6_test/launch/x6_base.launch.py"
    "src/myactuator_x6_test/launch/x6_moveit.launch.py"
)

for file in "${FILES[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓ $file${NC}"
    else
        echo -e "${RED}✗ Missing: $file${NC}"
    fi
done

echo ""
echo "Checking hardware interface configuration..."

# Check if EtherCAT driver is configured (not mock)
if grep -q "ethercat_driver/EthercatDriver" "src/x6_moveit_config/config/x6_test.ros2_control.xacro"; then
    echo -e "${GREEN}✓ Using EtherCAT driver (real hardware)${NC}"
else
    echo -e "${RED}✗ Still using mock hardware!${NC}"
    exit 1
fi

# Check controller names match
if grep -q "joint_trajectory_controller" "src/x6_moveit_config/config/moveit_controllers.yaml"; then
    echo -e "${GREEN}✓ Controller names match (joint_trajectory_controller)${NC}"
else
    echo -e "${RED}✗ Controller name mismatch${NC}"
    exit 1
fi

echo ""
echo "========================================="
echo -e "${GREEN}✓ All checks passed!${NC}"
echo "========================================="
echo ""
echo "Ready to run with real hardware:"
echo ""
echo "Terminal 1:"
echo "  cd /home/andrei-dragomir/ros2_eth"
echo "  source install/setup.bash"
echo "  ros2 launch myactuator_x6_test x6_base.launch.py"
echo ""
echo "Terminal 2:"
echo "  cd /home/andrei-dragomir/ros2_eth"
echo "  source install/setup.bash"
echo "  ros2 launch myactuator_x6_test x6_moveit.launch.py"
echo ""
echo "See MOVEIT_SETUP_GUIDE.md for detailed instructions."
echo ""
