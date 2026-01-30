#!/bin/bash

echo "=========================================="
echo "CDPR Simulation Launcher"
echo "=========================================="

# Exit on error
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to kill processes on exit
cleanup() {
    echo -e "\n${YELLOW}Cleaning up processes...${NC}"
    kill $GAZEBO_PID $CONTROLLER_PID $RVIZ_PID $VISUALIZER_PID 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "cdpr_controller" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true
    pkill -f "cdpr_visualizer" 2>/dev/null || true
    echo -e "${GREEN}Cleanup complete!${NC}"
    exit 0
}

# Set trap for cleanup
trap cleanup SIGINT SIGTERM EXIT

# Check required commands
echo -e "${YELLOW}Checking dependencies...${NC}"
for cmd in gz ros2 rviz2 python3; do
    if command_exists $cmd; then
        echo -e "  ${GREEN}✓${NC} $cmd"
    else
        echo -e "  ${RED}✗${NC} $cmd not found!"
        exit 1
    fi
done

# Build the package
echo -e "\n${YELLOW}Building CDPR package...${NC}"
cd ~/cdpr_ros2_ws
if colcon build --packages-select cdpr_control 2>/dev/null; then
    echo -e "  ${GREEN}Build successful!${NC}"
else
    echo -e "  ${RED}Build failed!${NC}"
    exit 1
fi

# Source the workspace
source install/setup.bash

# Create log directory
LOGS_DIR="$HOME/cdpr_logs"
mkdir -p $LOGS_DIR

echo -e "\n${YELLOW}Starting components...${NC}"

# 1. Start Gazebo
echo -e "  ${GREEN}1.${NC} Starting Gazebo..."
if command_exists gz; then
    gz sim ~/cdpr_ros2_ws/src/cdpr_control/worlds/cdpr_8cables_valid.sdf \
        --render-engine ogre2 \
        > $LOGS_DIR/gazebo.log 2>&1 &
    GAZEBO_PID=$!
    sleep 5
    echo -e "    Gazebo PID: $GAZEBO_PID"
else
    echo -e "    ${RED}Gazebo not found!${NC}"
    exit 1
fi

# 2. Start ROS 2 Controller
echo -e "  ${GREEN}2.${NC} Starting ROS 2 Controller..."
ros2 run cdpr_control cdpr_controller \
    > $LOGS_DIR/controller.log 2>&1 &
CONTROLLER_PID=$!
sleep 2
echo -e "    Controller PID: $CONTROLLER_PID"

# 3. Start RViz
echo -e "  ${GREEN}3.${NC} Starting RViz..."
rviz2 -d ~/cdpr_ros2_ws/src/cdpr_control/rviz/cdpr_visualization.rviz \
    > $LOGS_DIR/rviz.log 2>&1 &
RVIZ_PID=$!
sleep 3
echo -e "    RViz PID: $RVIZ_PID"

# 4. Start Python Visualizer (in virtual environment)
echo -e "  ${GREEN}4.${NC} Starting Python Visualizer..."
source ~/cdpr_ros2_ws/cdpr_venv/bin/activate
cd ~/cdpr_ros2_ws/src/cdpr_control
python3 scripts/cdpr_visualizer.py \
    > $LOGS_DIR/visualizer.log 2>&1 &
VISUALIZER_PID=$!
sleep 2
echo -e "    Visualizer PID: $VISUALIZER_PID"

# Monitor topics
echo -e "\n${YELLOW}Monitoring ROS 2 topics...${NC}"
echo -e "  To view topics: ros2 topic list"
echo -e "  To echo platform pose: ros2 topic echo /platform/pose"
echo -e "  To echo cable markers: ros2 topic echo /cable_markers"
echo -e "  To echo joint states: ros2 topic echo /joint_states"

echo -e "\n${GREEN}==========================================${NC}"
echo -e "${GREEN}All components started successfully!${NC}"
echo -e "${GREEN}==========================================${NC}"
echo -e "\n${YELLOW}Components running:${NC}"
echo -e "  1. Gazebo Simulation (PID: $GAZEBO_PID)"
echo -e "  2. ROS 2 Controller (PID: $CONTROLLER_PID)"
echo -e "  3. RViz Visualization (PID: $RVIZ_PID)"
echo -e "  4. Python Visualizer (PID: $VISUALIZER_PID)"
echo -e "\n${YELLOW}Log files are in:${NC} $LOGS_DIR"
echo -e "\n${YELLOW}Press Ctrl+C to stop all components${NC}"
echo -e "${YELLOW}==========================================${NC}"

# Keep script running
wait
