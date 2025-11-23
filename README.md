# AR4 ROS2 Stuff

This is a collection of ROS2 nodes & packages that I have created for different ways to control the AR4 robotic arm using ROS2.

It is mostly test code, so please excuse the lack of documentation.

Most nodes will have a README.md file with further information inside them.

### A quick overview of packages (see `src` folder):
#### `ar_face_detector`
Attach a camera the end of the arm and have it follow you around to keep your face in the center of the frame.

Watch on YouTube Shorts:

[![Watch on YouTube](https://img.youtube.com/vi/lzvzFQv-Cx0/0.jpg)](https://www.youtube.com/shorts/lzvzFQv-Cx0)


#### `ar_joy_control`
Control the robotic arm manually with an Xbox One Controller

Watch on YouTube Shorts:

[![Watch on YouTube](https://img.youtube.com/vi/easpZBXLTY4/0.jpg)](https://youtube.com/shorts/easpZBXLTY4)

#### `ar_whisper`
Command the robotic arm to move using voice commands

Watch on YouTube Shorts:

[![Watch on YouTube](https://img.youtube.com/vi/6ZTR-59qkAY/0.jpg)](https://youtube.com/shorts/6ZTR-59qkAY)

#### `esphome_leds`
ROS2 Node for controlling the LED strips installed on my robotic arm.

Watch on YouTube Shorts:

[![Watch on YouTube](https://img.youtube.com/vi/19Ep5-LoDiw/0.jpg)](https://youtube.com/shorts/19Ep5-LoDiw)

## Setup
Clone https://github.com/ycheng517/ar4_ros_driver to `lib` folder.

```
mkdir lib
git clone git@github.com:ycheng517/ar4_ros_driver.git lib/ar4_ros_driver
```

Run colcon build, but ignore the ar_hand_eye directory
```
colcon build --symlink-install --packages-ignore ar_hand_eye
```

## Run Moveit Test
### Pyhsical Hardware
```
ros2 launch ar_hardware_interface ar_hardware.launch.py calibrate:=True

ros2 launch ar_testing ar_moveit_test.launch.py use_sim_time:=False
```

### Gazebo
```
ros2 launch ar_gazebo ar_gazebo.launch.py

ros2 launch ar_testing ar_moveit_test.launch.py use_sim_time:=True
```