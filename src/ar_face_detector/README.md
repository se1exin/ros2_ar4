# Face Detector

The arm will follow you around and try to keep your face in the center of view.

Watch on YouTube Shorts:

[![Watch on YouTube](https://img.youtube.com/vi/lzvzFQv-Cx0/0.jpg)](https://www.youtube.com/shorts/lzvzFQv-Cx0)

## To run:

Start the arm
```
./launch-arm.sh
```

Start the mediapipe face detector node
```
ros2 run ar_face_detector ar_face_detector
```

Start the joint state publisher node
```
ros2 run ar_face_detector ar_face_detector_state_publisher
```