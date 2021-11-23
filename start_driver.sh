export ROS_MASTER_URI=http://ros-dist-ui:11311/
export ROS_IP=192.168.50.114
source ~/tormach_ws/devel/setup.bash
python3 tormach_driver.py --robot-info-file=config/tormach_za06_robot_default_config.yml
