# AR4 ROS2 Stuff

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