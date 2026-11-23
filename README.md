# RoboCup Keyboard Teleoperation

Ubuntu 24.04 with ROS 2 Jazzy, colcon, and rosdep. Connect the MCU with the robot firmware installed, power the motor drivers, and ensure your user has access to the serial port.

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
ros2 launch robocup_bringup omni_sim.launch.py \
  keyboard:=true joy:=false uart:=true serial_port:=/dev/ttyACM0
```

Replace `/dev/ttyACM0` with your MCU's serial port if needed. The launch opens Gazebo and the keyboard control window and automatically requests ARM.

Click the keyboard control window. Wait for `MCU ARMED`, then hold **Shift** while pressing motion keys.

| Key | Action |
|---|---|
| Shift + Up / Down | Forward / backward, up to 0.8 m/s |
| Shift + Left / Right | Left / right, up to 0.25 m/s |
| Shift + Q / E | Counterclockwise / clockwise, up to 0.6 rad/s |
| Space | Stop motion, keep ARM enabled |
| Esc | DISARM |
| A | Request ARM after a normal DISARM |
| Close window / Ctrl+C in the window | Request stop and DISARM, then exit |

Release Shift and motion keys before pressing A or resuming after Space. Releasing a motion key stops its axis. Losing window focus sends zero velocity. Press Ctrl+C in the launch terminal to shut down.
