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

Labyrinth Recognition
---------------------------------

<img src="https://raw.githubusercontent.com/hereissunyue/youbot_labyrinth/master/image/2.png">

The figure above demonstrated a rough segmentation of start region, goal region and obstacle wall. Rough processed map will produce much less stability for tracking part. We have to refine our rough map mathematically.

The basic idea is to divide the whole image into 31 X 31 grid regions. After resizing our map into appropriate size. We could calculate the mean value in certain region. With the average value, we could filter out noisy regions and assign value to the corresponging region with corrent pure unit value.

A part of MATLAB code for segmenting wall out of map could be done in the following way

```bash
% calculating the region status
for i = 1 : 31
    for j = 1 : 31
        sample = wall(10*(i-1)+1:10*i,10*(j-1)+1:10*j); % temp sample
        check = mean(mean(sample)); % calculate the average value in that region
        if check > 0.4
            wall_region = [wall_region; 10*(j-1)+1,10*(i-1)+1,10,10]; %#ok<AGROW>
        end
    end
end
wall_region_num = size(wall_region,1);
```

However, sometimes simply doing region-averaging might not get an accurate value, especially when we are doing the edge extraction, there is alway some regions will not work well. Hence, I dicided to used reference to build our map. We know our map is rotated with 0, 90, 180, 270 degrees. We could do simply image comparison and sort out the one with minimum difference. This method works very well.

We could also save the information in other format, like just with integer number range from 1 to 31. In such method, some future work might be easier. A decent result would be like the following:

<img src="https://github.com/hereissunyue/youbot_labyrinth/blob/master/image/2.GIF">


Path Planning
---------------------------------

A* algorithm is a very useful and easy implemented searching algorithm. With the help of Heuristic value, we could minimize the computation and time consuming to achieve real-time replanning.

With some algorithm to transfer the region into node list (bost avaibale path node list and obstacle list are needed for A* algorithm), we could implement our code and get the result in the following:

<img src="https://raw.githubusercontent.com/hereissunyue/youbot_labyrinth/master/image/3.png">


Ball Tracking with Kalman Filter
---------------------------------

Assume our color segmentation is always perfect and we could extract correctly our target ball successfully, then we don't need any more work in Tracking task. However, the reality is always cruel.

Our tracking job will encounter such problems:
1. The color segmentation will not work perfect even sometime not good at all, which might produce fake target even bigger than our target. This problem is mostly derived from lighting.
2. When the youBot arm is moving, the camera itself is shaking. With more frequent movement, the camera will acquire more un stable image, which would make our tacking accuracy much worse.
3. Camera image always contain noise, we need method to decrease the effect of them.
4. With extreme bad light situation, we might not have any measurement update at all. We need some probabilistic estimation to make our target stay in the frame. Becasue we know our target will always in the frame.

Kalman Filter is a very useful and easy implemented algorithm for probabilistic estimation, the basic algorithm is like

```bash
% Take Xt_1, Et_1, Zt, Rt, Qt, At, Ct and return Xt, Et
function [Xt, Et] = KalmanFilter(Xt_1, Et_1, Zt, Rt, Qt, At, Ct)
Xtba = At * Xt_1; % Prediction of State Vector
Etba = At * Et_1 * transpose(At) + Rt; % Prediction of State Covariance
Kt =  Etba * transpose(Ct) / (Ct * Etba * transpose(Ct) + Qt);  % Kalman Gain
Xt = Xtba + Kt * (Zt - Ct * Xtba); % Correction of State Vector:
I = eye(size(Et_1));
Et = (I - Kt * Ct) * Etba; % Correction of State Covariance
end
``` 

From the video clip, we could tell that our Kalman Filter is working although not that perfect. However, the tracking demo in the video is tracking a hard coded motion, and the camera bandwidth is not very high. However, with better equipment our tracking task will perform better for real-time control.



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

