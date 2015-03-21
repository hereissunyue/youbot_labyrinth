youbot_labyrinth
================

Package for youBot Labyrinth Solver Project.  

Director:  Jarvis Schultz

ME499 Winter. Northwestern University.



Project Introduction
============

The goal of this project is to use youBot to solve and plan the path of a labyrinth from start to goal based on computer vision technique, and then making execution of the youBot arm to make a small ball follow the trajectory we planned. In the original design of the project, during the movement of the ball, adjustment of dead reckoning error is also an important part of the project.

The project has two main parts, one of them is utilizing computer vision technique to recognize the labyrinth and planning the path with A* algorithm. The computer vision technique also contains objecting tracking task with Kalman Filter Estimation which is the foundation of youBot arm motion planning. 

The second part of this project is to a design controller of the youBot arm to do motion planning which makes the ball in the labyrinth following the trajectory we planned in computer vision part.

The progress so far is recorded in the video in the following:   [youBot Labyrinth Solver Video Clip](https://www.youtube.com/watch?v=Qshhv_lYqqU&feature=youtu.be)

<img src="https://raw.githubusercontent.com/hereissunyue/youbot_labyrinth/master/image/1.GIF">



Computer Vision Part
============

In this project, computer vision technique is introduced for two main purposes. The first one is labyrinth recognition and path planning. The second part is real-time tracking. We will discuss about how we achieve the task in detail in the following.

Labyrinth Recognition and Path Planning 
---------------------------------

Since the labyrinth is specially designed with black edge and white background, while the ball, start region and goal region are with R-G-B color respectively. We could easily doing color segmentation and basic image processing algorithm to extract our target and regions out.












The package dependancies include: 
---------------------------------

The following dependancies are clonable via github:

1) [hrl-kdl](https://github.com/gt-ros-pkg/hrl-kdl)

2) [urdfdom](https://github.com/ros/urdfdom)

3) [brics_actuator](http://wiki.ros.org/brics_actuator) ( The brics messages are required by the hrl-kdl) 

The package also relies on the following packages, installable from apt-get in Hydro. 

4) [youbot_driver](https://github.com/youbot/youbot_driver) 

5) [youbot_driver_ros_interface](https://github.com/youbot/youbot_driver_ros_interface)


Running the Package
================

The youBot arm must be powered on, and the arm must be initialized. Then we simply need to launch the main launch file using

```bash
roslaunch youbot_grasp_msr grasp.launch
```
The inverse and forward kinematics of the arm are calculated in order to drive the arm to a predefined position and grasp a square block. The arm then returns to a predefined pose and then drops the block back where it picked it up.

Additional Information
----------------------

This package is part of a larger project developed by a group of students in the [Northwestern MSR](http://robotics.northwestern.edu/) program. The completed project will be able to command the youBot to grasp an object that is identified by a laser pointer. It will navigate to the object using the [youbot_nav_msr](https://github.com/jihoonkimMSR/youbot_nav_msr) package and determine a precise grasp point using an [ASUS Xtion PRO LIVE](http:/www.asus.com/us/Multimedia/Xtion_PRO_LIVE) attached at the arm to provide visual feedback for grasping.  The object will then be carried by the youBot to a drop-off location.

Note of caution
---------------

The youBot arm is powerful, fast, and does not have any built-in collision detection. Be cautious in choosing desired gripper positions to avoid any damage to the youBot and/or users. 

Resources
=========

Here are a few links that may be useful in working with the youBot and its inverse kinematics: 

The [youbot_arm_test](https://github.com/youbot/youbot_driver_ros_interface) is a good place to get started with controlling the youBot arm.  In particular [this file](https://github.com/youbot/youbot_driver_ros_interface/blob/hydro-devel/src/examples/youbot_arm_test.cpp) provides good information for publishing joint positions comands in C++.

The authors of this package also found [youbot_teleop](https://github.com/adamjardim/youbot_teleop) a helpful tool during the development and testing of the youbot_grasp_msr package. 

[Unified Closed Form Inverse Kinematics for the KUKA youBot](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?reload=true&arnumber=6309496) outlines a nice technique for solving the IK problem for the youBot with full 8-DOF youBot (3 from the base and 5 from the arm). The developers of youbot_grasp_msr wish to implement the method proposed in this paper in the near future. 

[youbot Manipulation](https://github.com/svenschneider/youbot-manipulation): Sven Schneider has some really good examples of solving the IK problems for the youBot. These examples have been largely deprecated by the release of newer versions of ROS, and the transition from the [ROS arm-navigation](http://wiki.ros.org/arm_navigation) stack to [MoveIt!](http://moveit.ros.org/). 

Acknowledgements
================

Thank you to [Jarvis Schultz](https://github.com/jarvisschultz) for contributing the ```dls_ik```, ```dls_ik_position_only```, and ```inverse_biased``` functions and their required utility functions in youbot_grasping_kdl.py.

