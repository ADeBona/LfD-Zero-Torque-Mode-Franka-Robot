# Franka Setup Guide 💪💻

Welcome to the **Franka Installation Tutorial**! This guide walks you through setting up the Franka Emika Panda robot for your Ubuntu system. By the end, you'll have:

- **Libfranka** installed 
- **franka_ros** configured 
- **Cartesian Impedance Control** enabled

## ✨ Libfranka Installation

### Steps

1. **Identify Your Franka Controller Version:**
   - Open the Franka Desk interface: [https://172.16.0.2/desk](https://172.16.0.2/desk).
   - Click the three lines in the top-right corner for the dropdown menu.
   - Select **Settings** to view your Franka version (e.g., `4.0.3`, `2.1.1`).

2. **Check Compatibility:**
   - Visit [Franka Compatibility Docs](https://frankaemika.github.io/docs/compatibility.html) to find the correct versions of `libfranka` and `franka_ros` for your system.
   - Example: For **ROS Kinetic** and **Franka version 2.1.1**, use `libfranka 0.5.0` and `franka_ros 0.6.0`.

3. **Install `libfranka` from Source:**
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

### Success 🎉
Congrats! You’ve installed `libfranka`. Let’s move on to **franka_ros**. 🚀

---

## 🚀 franka_ros Installation

### Steps

1. **Set Up Your Catkin Workspace:**
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

4. **Reboot Your System.**

### Testing 🔧
Run the following commands to verify:
```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch franka_control franka_control.launch robot_ip:=172.16.0.2
```
- Use `rostopic list` to view available topics.
- **Common Issue:** If you see `[ERROR]: libfranka: Move command aborted`:
  - Ensure `libfranka` and `franka_ros` versions match.
  - Release the emergency stop (Franka lights should be blue).

---

## 🪼 Cartesian Impedance Control (Zero Torque Mode)

### Steps

1. **Modify Configuration Files:**
   Replace files in the `franka_example_controllers` package with the ones provided in this repository:
   - `config/franka_example_controllers.yaml`
   - `cfg/compliance_param.cfg`
   - `include/pseudo_inversion.h`
   - And more...

   To automate this:
   
   1. **Make the Script Executable:**
      ```bash
      chmod +x setup_cartesian_impedance.sh
      ```

   2. **Run the Script:**
      ```bash
      ./setup_cartesian_impedance.sh
      ```

2. **Build the Workspace:**
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. **Launch MoveIt Configuration:**
   ```bash
   roslaunch franka_example_controllers franka_moveit.launch
   ```

4. **Enable Cartesian Impedance Control:**
   Open a new terminal and run:
   ```bash
   cd ~/catkin_ws/src/franka_ros/franka_example_controllers/script
   chmod +x data_collection_top.py
   rosrun franka_example_controllers data_collection_top.py
   ```

### Notes
- Press **`h`** to move the robot to its home position.
- Press **`t`** to enable the zero torque Cartesian impedance controller for manual movement.

---

## 📊 Recording ROSBAG Data

Steps

Set Up for ROSBAG Recording:
Identify the topics you want to record. Common topics for the Franka Emika Panda include:

/franka_state_controller/F_ext (External forces)

/franka_state_controller/joint_states (Joint positions, velocities, and efforts)

/cartesian_impedance_example_controller/ee_pose (End-effector position and orientation)

/cartesian_impedance_example_controller/ee_twist (End-effector velocity)

Start Recording Data:
Use the following command to record the desired topics:

rosbag record -o $(date +"franka_data_%Y-%m-%d_%H-%M-%S") /franka_state_controller/F_ext /franka_state_controller/joint_states /cartesian_impedance_example_controller/ee_pose /cartesian_impedance_example_controller/ee_twist

This will generate a .bag file with a timestamped name (e.g., franka_data_2024-12-27_14-45-30.bag) in the current directory where the command is executed. You can later analyze or replay it for debugging and visualization. 📈

Stop Recording:
Press Ctrl+C in the terminal where the rosbag command is running to stop recording.

Verify the Data:
Use the following command to inspect the contents of the recorded bag file:

rosbag info franka_data_*.bag

Notes

Ensure the robot is operational and publishing data on the desired topics before starting the recording.

The timestamp in the filename makes it easy to organize recordings by date and time.

ROSBAG files can be used for offline analysis or playback of recorded robot states.

---

## 🌟 Conclusion

That’s it! You’re now ready to explore the full potential of the **Franka Emika Panda** robot. Whether you’re tinkering with Cartesian Impedance Control or simply learning the basics, we hope this guide empowers you to achieve your goals.

Happy coding, and enjoy working with your robot! 🛠️✨

