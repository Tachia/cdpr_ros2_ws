#!/bin/bash

echo "Starting CDPR Simulation..."

# 1. Build the package
echo "Building package..."
cd ~/cdpr_ros2_ws
colcon build --packages-select cdpr_control

# 2. Source the workspace
source install/setup.bash

# 3. Open multiple terminals for different components
echo "Opening simulation components..."

# Terminal 1: Gazebo
gnome-terminal -- bash -c "echo 'Starting Gazebo...'; gz sim ~/cdpr_ros2_ws/src/cdpr_control/worlds/cdpr_8cables_valid.sdf; exec bash"

# Wait for Gazebo to start
sleep 5

# Terminal 2: ROS 2 Controller
gnome-terminal -- bash -c "echo 'Starting CDPR Controller...'; source ~/cdpr_ros2_ws/install/setup.bash; ros2 run cdpr_control cdpr_controller; exec bash"

# Terminal 3: RViz
gnome-terminal -- bash -c "echo 'Starting RViz...'; source ~/cdpr_ros2_ws/install/setup.bash; rviz2 -d ~/cdpr_ros2_ws/src/cdpr_control/rviz/cdpr_visualization.rviz; exec bash"

# Terminal 4: Topic monitoring
gnome-terminal -- bash -c "echo 'Monitoring topics...'; source ~/cdpr_ros2_ws/install/setup.bash; ros2 topic list; echo 'Press Ctrl+C to exit'; ros2 topic echo /platform/pose; exec bash"

# Terminal 5: Python visualizer
gnome-terminal -- bash -c "echo 'Starting Python Visualizer...'; source ~/cdpr_ros2_ws/install/setup.bash; cd ~/cdpr_ros2_ws/src/cdpr_control; python3 scripts/cdpr_visualizer.py; exec bash"

echo "All components started!"
echo "Check the opened terminals for visualization."
