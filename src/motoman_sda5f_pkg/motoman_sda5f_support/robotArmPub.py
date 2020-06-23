#!/usr/bin/env python  
'''
Simulation in RVIZ: 
Get arm config data and publish whole body config of SDA5f. 
'''
import rospy 
import rospkg 
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np

def pubJoint(armDataL, armDataR):
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz
    wholeJoints = JointState()
    wholeJoints.header = Header()
    wholeJoints.name = ['torso_joint_b1', 'torso_joint_b2', 'arm_left_joint_1_s', 'arm_left_joint_2_l', 'arm_left_joint_3_e',
                        'arm_left_joint_4_u', 'arm_left_joint_5_r', 'arm_left_joint_6_b', 'arm_left_joint_7_t',
                        'arm_right_joint_1_s', 'arm_right_joint_2_l', 'arm_right_joint_3_e', 'arm_right_joint_4_u',
                        'arm_right_joint_5_r', 'arm_right_joint_6_b', 'arm_right_joint_7_t']
    wholeJoints.velocity = []
    wholeJoints.effort = []                
    
    tTotal = np.size(armDataL,0)
    qArmL = np.zeros((7,), dtype=float)
    qArmR = np.zeros((7,), dtype=float)
    qTorso2 = [0.0, 0.0]

    while not rospy.is_shutdown():
        for i in range(tTotal):
            qArmL = armDataL[i,:]
            qArmR = armDataR[i,:]
            qArmR7 = qArmR.tolist()
            qArmL7 = qArmL.tolist()
            wholeJoints.position = qTorso2 + qArmL7 + qArmR7
            wholeJoints.header.stamp = rospy.Time.now()
            pub.publish(wholeJoints)
            rate.sleep()

if __name__ == '__main__':
    armDataL = np.loadtxt("/home/ang/Downloads/ikPinoSda5f/data/qArmLeft.txt", dtype=float)
    armDataR = np.loadtxt("/home/ang/Downloads/ikPinoSda5f/data/qArmRight.txt", dtype=float)
    try:
        pubJoint(armDataL, armDataR)

    except rospy.ROSInterruptException:
        pass

