# Joystick Control Node for AR4 Robot Arm

Control the AR4 Robot Arm with an Xbox One Controller

## Requirements
An Xbox One Controller with USB cable that connects to your computer.

Make sure the `joy` package is installed:

```
sudo apt install ros-iron-joy
```

## Running
One a terminal, start the arm (e.g. `./launch-arm.sh`).

In another terminal, start the joystick node:
```
ros2 run joy joy_node --ros-args -p sticky_buttons:=True
```

In a third terminal, start the `ar_joy_control` node:
```
ros2 run ar_joy_control ar_joy_control
```

## Controlling the arm
Currently, the left thumbstick maps to J1 and J2.

The right thumbstick maps to J3 and J4, BUT if you press the 'B' button on the Xbox controller, the right thumb changes to J5 and J6.

The 'A' button opens and closes the gripper.


