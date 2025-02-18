# Experimental Robotics Laboratory - Assignment 1 ROS2 Aruco Recognition Using OpenCV

In this project a mobile robot is that rotates and scans 5 aruco markers and drawas a circle aroun dthem and publishes the image.

This project is developed by:
 *Ouassim Milous - s5938924*


## Dependencies

1. **Aruco:**  - [OpenCV Aruco Marker Tracking GitHub Repository](https://github.com/carmineD8/ros2_aruco)


## Installation

- Clone the packages required for the project inside src folder of your ros2 workspace
```
git clone https://github.com/OuassimMilous/Experimental-Robotics-Lab---ROS2-Open-CV-ArUco-recognition
git clone https://github.com/carmineD8/ros2_aruco
```

- Install the dependencies of the packages and build your workspace
```
colcon build
```
-  Source your workspace.
```
source install/setup.bash
```

## Running the project

- For the robot rotation:
```
ros2 launch robot_urdf gazebo_fixed.launch.py
```
- For the camera rotation:
```
ros2 launch robot_urdf gazebo_rotated.launch.py
```
