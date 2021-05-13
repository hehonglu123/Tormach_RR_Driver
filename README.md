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
* Build custom ROS messages in a separate ROS workspace (one-time only), and source it
```
  source ~/tormach_ws/devel/setup.bash
```
* Start Pathpilot and Docker running on Tormach control computer

Double click `PathPilot` application, then
```
docker exec -itu 1000:1000 ros-dist-ui bash -i
source /opt/ros/noetic/setup.bash
```
* Set up ROS communication protocol on RR driver PC
```
sudo nano /etc/hosts
```
Add `ros-dist-ui` and the IP of Tormach computer in the host book, then
```
  export $ROS_MASTER_URI=http://ros-dist-ui:11311/
  export $ROS_IP=<IP of PC>
```
Make sure rostopics/services are visible after the settings.
* Start RR driver
```
python tormach_driver.py --robot-info-file=tormach_za06_robot_default_config.yml
```

### Running RR client
* Jog joint position & Position command example
```
python tormach_client1.py
```
* Jog joint velocity & Trajectory command example
```
* python tormach_client2.py
```
