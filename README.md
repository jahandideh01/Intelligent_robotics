# Intelligent_robotics
writing a robot with ROS noetic

this is only answer of the assignment
you should follow instruction that how to install reauirements and download needed files, then add this repository to catkin_ws/src directory.
then build it...


# IR2425 Group 16 - Intelligent Robotics Assignment

##  Package name
`ir2425_group_16`

##  Nodes

- **node_a** → Calls service and publishes AprilTag IDs.
- **node_b** → Subscribes to IDs and logs them.
- **coeffs_service_server** → Dummy service providing coefficients.

##  How to build

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash

 How to run

roslaunch ir2425_group_16 assignment.launch

 Check topics

rostopic echo /tag_ids

 Service

rosservice call /apriltags_ids_srv "ready: true"

