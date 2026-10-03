#!/bin/bash
set -e
echo "Starting micro-ROS Agent setup..."
# 1. Install prerequisites in case they are missing
echo "Installing prerequisites..."
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool git build-essential
# Initialize rosdep if not already done
sudo rosdep init 2>/dev/null || true
rosdep update
# 2. Setup micro-ROS Agent Workspace
echo "Creating workspace..."
mkdir -p ~/microros_ws/src
cd ~/microros_ws
# Clone setup package
echo "Cloning micro-ROS setup..."
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
# Source ROS 2 installation (Ensure ROS 2 is accessible in this shell)
source /opt/ros/humble/setup.bash
# Install dependencies using rosdep
echo "Installing package dependencies..."
rosdep install --from-paths src --ignore-src -y
# 3. Build the micro_ros_setup package
echo "Building micro_ros_setup..."
colcon build
source install/local_setup.bash
# 4. Create the agent workspace and build the agent
echo "Creating and building the micro-ROS agent..."
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
# 5. Add source command to bashrc so it's available on every login
if ! grep -q "source ~/microros_ws/install/local_setup.bash" ~/.bashrc; then
    echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
fi
echo "========================================================"
echo "✅ micro-ROS Agent Setup Complete!"
echo "Please run: source ~/.bashrc"
echo ""
echo "To run the agent over Wi-Fi (UDP):"
echo "  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
echo "========================================================"
