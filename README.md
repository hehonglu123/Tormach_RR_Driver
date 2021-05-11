# Tormach ZA06 Robot Raconteur Driver (Through ROS API)
Robot Raconteur is an object oriented robot communication library. This driver utilize [standard RR robot definition.](https://github.com/robotraconteur/robotraconteur_standard_robdef/blob/master/group1/com.robotraconteur.robotics.robot.robdef)

## Prerequisite (Driver):
* Ubuntu
* Python
* [Robot Raconteur](https://github.com/robotraconteur/robotraconteur/wiki/Download) 
* [ROS 1](http://wiki.ros.org/noetic/Installation) 

## Prerequisite (Client):
* [Robot Raconteur](https://github.com/robotraconteur/robotraconteur/wiki/Download) (Follow instruction to download, depending on different OS or programming environment)

## Instructions:

### Running RR driver
In order to start Robot Raconteur driver, 
* Build custom ROS messages in a separate ROS workspace
* Set up ROS communication protocol (eg.
