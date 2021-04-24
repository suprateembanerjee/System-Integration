# System Integration
 This is the Capstone project of Udacity Self Driving Car Engineer Nanodegree
 
 The project uses the **simulator** developed by Udacity which can be downloaded from [here](https://github.com/udacity/CarND-Capstone/releases).

---

## Dependencies

* **Ubuntu 16.04 Xenial Xerus**[download](https://releases.ubuntu.com/16.04/?_ga=2.188039087.669715270.1619243707-2109527022.1619243707)
* **ROS Kintetic** [download](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* **Python Dependencies**  
`pip install -r requirements.txt`

Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator:

|        | Simulator |
| :-----------: |:-------------:|
| Nvidia driver | 384.130 |
| CUDA | 8.0.61 |
| cuDNN | 6.0.21 |
| TensorRT | N/A |
| OpenCV | 3.2.0-dev |
| OpenMP | N/A |

## Usage

* Setup OS

  If not using a native Ubuntu 16.04 environment, use [Virtualbox](https://www.virtualbox.org/wiki/Downloads) to mount the Ubuntu guest image, and install ROS Kintetic.

* Execute the following commands in a terminal window:
```
cd System-Integration
pip install -r requirements.txt
cd ros
catkin_make
source devel/setup.sh
roslaunch launch styx.launch
```
The program should connect to port `4567`. Launching the simulator at this point would automatically connect to the port.

* Disable Manual Mode and turn Camera on
  
  In the Simulator, `Manual` mode must be unchecked and `Camera` must be turned on for Traffic Light Detection.
  
Once these steps are followed, the project should work as intended, i.e. drive the car, change lanes as necessary, and stop at traffic lights if they are Red.

[//]: # (Image References)

[image1]: ./imgs/final-project-ros-graph-v2.png "ROS Graph"
[image2]: ./imgs/unity.png "Sample Simulator Image"

## ROS Architecture

![alt text][image1]

### Node: Traffic Light Detection

This node is described in [this](https://github.com/suprateem48/System-Integration/blob/main/ros/src/tl_detector/tl_detector.py) file.

The node subscribes to `/base_waypoints`, `/image_color`, and `/current_pose` topics, and publishes the locations to stop for red traffic lights to the `/traffic_waypoint` topic.

The actual traffic light classifier has not been built yet, but provisions for building them has been built into existing code. Currently, the node additionally subscribes to `/vehicle/traffic_lights` to get surrounding traffic light information.

### Node: Waypoint Updater

This node is described in [this](https://github.com/suprateem48/System-Integration/blob/main/ros/src/waypoint_updater/waypoint_updater.py) file.

The node subscribes to `/base_waypoints`, `/obstacle_waypoints`, `/traffic_waypoint` and `/current_pose` topics, and publishes a list of waypoints ahead of the car with target velocities to the `/final_waypoints` topic.

### Node: Drive By Wire (DBW)

This node is described in [this](https://github.com/suprateem48/System-Integration/blob/main/ros/src/twist_controller/dbw_node.py) file.

The node subscribes to `/current_velocity`, `/twist_cmd`, and `/vehicle/dbw_enabled` topics, and publishes brake, throttle and steering control commands to `/vehicle/brake_cmd`, `/vehicle/throttle_cmd` and `/vehicle/steering_cmd` topics respectively.

This node is extensively aided by another file in the same package titled [twist_controller.py](https://github.com/suprateem48/System-Integration/blob/main/ros/src/twist_controller/twist_controller.py), which contains the logic behind much of the control system. It utilizes other files in the same package to use [PID](https://github.com/suprateem48/System-Integration/blob/main/ros/src/twist_controller/pid.py) and [Yaw](https://github.com/suprateem48/System-Integration/blob/main/ros/src/twist_controller/yaw_controller.py) Controllers, and [LowPass](https://github.com/suprateem48/System-Integration/blob/main/ros/src/twist_controller/lowpass.py) Filters.

### Package: Styx

This package is described [here](https://github.com/suprateem48/System-Integration/tree/main/ros/src/styx).

This package contains a server for communicating with the simulator, and a bridge to translate and publish simulator messages to ROS topics.

### Package: Styx Messages

This package is described [here](https://github.com/suprateem48/System-Integration/tree/main/ros/src/styx_msgs).

This package includes definitions of the custom ROS message types used in the project.

### Package: Waypoint Loader

This package is described [here](https://github.com/suprateem48/System-Integration/tree/main/ros/src/waypoint_loader).

This package loads the static waypoint data and publishes to `/base_waypoints`.

### Package: Waypoint Follower

This package is described [here](https://github.com/suprateem48/System-Integration/tree/main/ros/src/waypoint_follower).

This package contains code from [Autoware](https://github.com/Autoware-AI/autoware.ai) which subscribes to `/final_waypoints` and publishes target vehicle linear and angular velocities in the form of twist commands to the `/twist_cmd` topic.

## Output

Example frame from simulator:
![alt text][image2]
