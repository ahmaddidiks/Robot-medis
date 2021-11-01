#!/usr/bin/env python3

import rospy
from dynamixel_control.msg import DynamixelPosList, DynamixelPos
from sensor_msgs.msg import JointState
import numpy as np

rospy.init_node('forward_kineamtics')
dynamixelPosList_pub = rospy.Publisher('set_position', DynamixelPosList, queue_size=10)

def fk(joint):
    data_pub = DynamixelPosList()
    id = 1
    position = joint.position
    position = np.degrees(position)
    for p in position:
        pos = DynamixelPos()
        pos.id = id
        id +=1
        pos.position = int(p+180) #manipulasi dengan menambah 180 deg
        data_pub.dynamixel.append(pos)
    
    dynamixelPosList_pub.publish(data_pub)

if __name__ == '__main__':
    rospy.Subscriber('joint_states', JointState ,fk)
    rate = rospy.Rate(10)
    rospy.spin()