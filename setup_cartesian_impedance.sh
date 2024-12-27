#!/bin/bash

# Automated setup for Cartesian Impedance Control - Zero Torque Mode

# Define paths
FRANKA_EXAMPLE_CONTROLLERS_PATH=~/catkin_ws/src/franka_ros/franka_example_controllers
PANDA_MOVEIT_CONFIG_PATH=~/catkin_ws/src/franka_ros/panda_moveit_config

# Check if franka_example_controllers exists
if [ ! -d "$FRANKA_EXAMPLE_CONTROLLERS_PATH" ]; then
    echo "Error: franka_example_controllers package not found in $FRANKA_EXAMPLE_CONTROLLERS_PATH."
    exit 1
fi

# Replace necessary files
echo "Replacing configuration and source files for Cartesian Impedance Control..."
cp ./config/franka_example_controllers.yaml "$FRANKA_EXAMPLE_CONTROLLERS_PATH/config/"
cp ./cfg/compliance_param.cfg "$FRANKA_EXAMPLE_CONTROLLERS_PATH/cfg/"
cp ./include/pseudo_inversion.h "$FRANKA_EXAMPLE_CONTROLLERS_PATH/include/"
cp ./src/cartesian_impedance_example_controller.cpp "$FRANKA_EXAMPLE_CONTROLLERS_PATH/src/"

# Add panda_moveit_config package if not already present
if [ ! -d "$PANDA_MOVEIT_CONFIG_PATH" ]; then
    echo "Adding panda_moveit_config package..."
    cp -r ./panda_moveit_config "$FRANKA_EXAMPLE_CONTROLLERS_PATH/../"
else
    echo "panda_moveit_config package already exists."
fi

# Compile the workspace
echo "Compiling the workspace..."
cd ~/catkin_ws
catkin_make || { echo "Error: catkin_make failed."; exit 1; }

# Source the workspace
echo "Sourcing the workspace..."
source devel/setup.bash

# Copy the launch file
LAUNCH_FILE_PATH="$FRANKA_EXAMPLE_CONTROLLERS_PATH/launch/franka_moveit.launch"
if [ ! -f "$LAUNCH_FILE_PATH" ]; then
    echo "Adding franka_moveit.launch file..."
    cp ./launch/franka_moveit.launch "$FRANKA_EXAMPLE_CONTROLLERS_PATH/launch/"
else
    echo "franka_moveit.launch already exists."
fi

# Copy the Python script
PYTHON_SCRIPT_PATH="$FRANKA_EXAMPLE_CONTROLLERS_PATH/script/data_collection_top.py"
if [ ! -f "$PYTHON_SCRIPT_PATH" ]; then
    echo "Adding data_collection_top.py script..."
    cp ./script/data_collection_top.py "$FRANKA_EXAMPLE_CONTROLLERS_PATH/script/"
else
    echo "data_collection_top.py script already exists."
fi

# Make the Python script executable
chmod +x "$PYTHON_SCRIPT_PATH"

echo "Setup complete. You can now run the controller using the following commands:"
echo "1. roslaunch franka_example_controllers franka_moveit.launch"
echo "2. In a new terminal, run: rosrun franka_example_controllers data_collection_top.py"
