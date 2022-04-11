import rospy, time
from control_msgs.msg import JointJog

DEFAULT_AUTOREPEAT_INTERVAL = 250
JOG_COMMAND_TOPIC = 'jog_arm_server/joint_delta_jog_cmds'

rospy.init_node('node_name')
pub = rospy.Publisher(JOG_COMMAND_TOPIC, JointJog, queue_size=1)
r = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    jj = JointJog()
    jj.header.stamp = rospy.Time.now()
    joint_names=['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
    for name in joint_names:
        jj.joint_names.append(name)
        value = float(-0.5)
        jj.velocities.append(value)
    pub.publish(jj)
    r.sleep()

