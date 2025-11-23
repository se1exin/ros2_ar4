# SAM Detector

Run the arm
```
ros2 launch annin_ar4_driver driver.launch.py calibrate:=True ar_model:="mk2"
```


Run the SAM Listener -> Moveit commander node
```
ros2 launch sam_detector run.launch.py
```

Run the SAM Detector node
```
ros2 run sam_detector sam_detector
```