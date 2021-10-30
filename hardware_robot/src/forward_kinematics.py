import rospy
from dynamixel_control.msg import DynamixelPostList, DynamixelPos
from sensor_msgs.msg import JointState
import numpy as np

rospy.init_node('forward_kineamtics')
dynamixelPosList_pub = rospy.Publisher('set_position', DynamixelPostList, queue_size=10)

def fk(joint):
    data_pub = DynamixelPostList()
    id = 1
    position = joint.position
    position = np.degrees(position)
    
    for p in position:
        pos = DynamixelPos()
        pos.id = id
        id +=1
        pos.position = p
        data_pub.append(pos)
    
    dynamixelPosList_pub.publish(data_pub)

if __name__ == '__main__':
    rospy.Subscriber('joint_states', JointState ,fk)