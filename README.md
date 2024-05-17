# AR4 ROS2 Stuff

## Setup
Clone https://github.com/ycheng517/ar4_ros_driver to `lib` folder.

```
mkdir lib
git clone git@github.com:ycheng517/ar4_ros_driver.git lib/ar4_ros_driver
```



## Run Moveit Test
```
ros2 launch ar_testing ar_moveit_test.launch.py use_sim_time:=True
```
