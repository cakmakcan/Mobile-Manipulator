# Mobile-Manipulator

## Introduction:
This simulation has been tested with Melodic version of ROS with ubuntu 18.04 !

This repository has been crated for thesis purposes. I created a mobile manipulator model contains Robotnik Summit XL Steel mobile robot, which repository can also be found here :https://github.com/RobotnikAutomation/summit_xl_sim

Also for manipulation, I used Panda Robot and the repository is here for any cases : https://github.com/justagist/panda_simulator

Using both these two packages, I created a new uniqe model to use mobile manipulator for field applications.

## Installation and Intructions:

To facilitate the installation you can use the vcstool:
```
sudo apt-get install -y python3-vcstool
```

Create a workspace and clone the repository:
```
mkdir catkin_ws
cd catkin_ws
```

Install the ROS dependencies:
```
rosdep install --from-paths src --ignore-src --skip-keys="summit_xl_robot_control marker_mapping robotnik_locator robotnik_pose_filter robotnik_gazebo_elevator" -y -r
```
Compile:

```
catkin build
source devel/setup.bash
```
ONLY: if catkin build doesn't work: The package catkin-tools is need to compile with catkin build:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools
```

Launch Summit XL simulation:

```
roslaunch summit_xl_sim_bringup summit_xl_complete.launch
```

Launch MoveIt:

```
roslaunch summit_xl_panda_moveit_config robot_execution.launch
```

Run control program:
 
```
rosrun summit_xl_action fsm.py
```
