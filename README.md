[Research Track I](https://corsi.unige.it/en/off.f/2021/ins/51201)<br>
**Programmer:** [Ankur Kohli](https://github.com/ankurkohli007)<br>
[M.Sc Robotics Engineering](https://corsi.unige.it/corsi/10635)<br>
[University of Genoa (UniGe)](https://unige.it/en)<br>
**Supervisor:** [Prof. Carmine Tommaso Recchiuto](https://rubrica.unige.it/personale/UkNDWV1r)

# Assignment 3: Software Architecture for Mobile Robot Control

## Abstract
This assignment is the ***development of a software architecture for the control of a mobile robot***. This is also based on how to control the robot using [ROS](https://www.ros.org/) (Robot Operating System), [Gazebo and RViz](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros). The assignment consists in writing ROS nodes: a controller for a robot, and a UI. For this, ***Python*** programing is used. 

## Introduction
A robot moves in an environment initially unknown to it. The software architecture is developed to control the robot. The software will rely on the *move_base and gmapping packages* for localizing the robot and plan the motion. The architecture should be able to get the user request, and let the robot execute one of the following behaviors (depending on the user’s input) as follows:

* Autonomously reach a x,y coordinate inserted by the user
* Let the user drive the robot with the keyboard, 
* Let the user drive the robot assisting them to avoid collisions. 

Thanks to laser scanners mounted on board the robot, it is able to build a map of the environment and move without encountering obstacles. 

## Objective
The architecture should be able to get the user request, and let the robot execute robot behvaiour such as ***autonomous drive***, ***manual drive using teleop***, and ***manual drive using teleop and avoiding the collisions***. 
