# README

## About

This tutorial explains how to install Franka for your Ubuntu system. It includes instructions to set up:

- Libfranka
- franka_ros
- Cartesian Impedance Control

This repository is developed by Alessio De Bona and is inspired by Venkatesh's original tutorial. It has been tested on:
- **Ubuntu 16.04** (ROS Kinetic)
- **Ubuntu 20.04** (ROS Noetic)

### Note
This repository contains a single Python script to verify the working of the setup.

## Libfranka Installation

### Steps
1. **Identify your Franka Controller Version:**
   - Open the Franka Desk interface: [https://172.16.0.2/desk](https://172.16.0.2/desk).
   - Click on the three lines in the top-right corner to open the dropdown menu.
   - Select `Settings` to view the Franka version (e.g., `4.0.3`, `2.1.1`).

2. **Check Compatibility:**
   - Visit [Franka Compatibility Docs](https://frankaemika.github.io/docs/compatibility.html) to determine the compatible versions of `libfranka` and `franka_ros`.
   - Example: For **ROS Kinetic** and **Franka version 2.1.1**, the compatible versions are `libfranka 0.5.0` and `franka_ros 0.6.0`.

3. **Install `libfranka` from source:**
   ```bash
   cd
   sudo apt remove "*libfranka*"
   sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
   git clone --recursive https://github.com/frankaemika/libfranka
   cd libfranka
   git checkout <version>
   git submodule update
   mkdir build && cd build
   cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
   cmake --build .
   ```

### Success
Congratulations! You have successfully installed `libfranka`. Proceed to the next step.

---

## franka_ros Installation

### Steps
1. **Set up your Catkin Workspace:**
   ```bash
   cd
   mkdir -p catkin_ws/src
   cd catkin_ws
   source /opt/ros/noetic/setup.sh
   catkin_init_workspace src
   ```

2. **Clone and Build `franka_ros`:**
   ```bash
   git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
   cd src/franka_ros
   git checkout <version>
   rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
   cd ~/catkin_ws
   catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
   source devel/setup.sh
   ```

3. **Configure Real-Time Permissions:**
   ```bash
   sudo addgroup realtime
   sudo usermod -a -G realtime $(whoami)
   ```

4. **Reboot your system.**

### Testing
To check the setup:
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch franka_control franka_control.launch robot_ip:=172.16.0.2
```

- Use `rostopic list` to view topics.
- If you encounter `[ERROR]: libfranka: Move command aborted`, ensure:
  - `libfranka` and `franka_ros` versions match.
  - Emergency stop is released (Franka lights should be blue).

---

## Cartesian Impedance Control (Zero Torque Mode)

### Steps
1. **Modify Configuration Files:**
   Replace the following files in the `franka_example_controllers` package with the ones provided in this repository:
   - `config/franka_example_controllers.yaml`
   - `cfg/compliance_param.cfg`
   - `include/pseudo_inversion.h`
   - `src/cartesian_impedance_example_controller.cpp`

2. **Add `panda_moveit_config`:**
   Ensure the `panda_moveit_config` folder is present in `~/catkin_ws/src/franka_ros/`.

3. **Build the Workspace:**
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. **Launch MoveIt Configuration:**
   Add `franka_moveit.launch` to the launch folder (e.g., `~/catkin_ws/src/franka_ros/franka_example_controllers/launch`) and run:
   ```bash
   roslaunch franka_example_controllers franka_moveit.launch
   ```

5. **Enable Cartesian Impedance Control:**
   Add `data_collection_top.py` to the `script` folder (e.g., `~/catkin_ws/src/franka_ros/franka_example_controllers/script`) and run:
   ```bash
   cd ~/catkin_ws/src/franka_ros/franka_example_controllers/script
   chmod +x data_collection_top.py
   rosrun franka_example_controllers data_collection_top.py
   ```

### Notes
- Press **`h`** to move the robot to its home position.
- Press **`t`** to enable the zero torque Cartesian impedance controller, allowing free manual movement.

---

## Conclusion

This repository provides all the tools and configurations required to set up Franka Emika Panda for ROS. Happy coding and enjoy working with your robot!
