#!/usr/bin/env python
import rospy, math
import numpy as np
from time import sleep
from std_msgs.msg import Bool, UInt8
from dynamixel_control.msg import DynamixelPosList
from inverse_kinematics.msg import FullBodyIK
from motion_control.msg import walkParam

rospy.init_node('motion_control_node')
fullBodyIK_pub = rospy.Publisher('full_body_target', FullBodyIK, queue_size=10)

feet_height = 24.0	# cm
walk_height = 5.0   	# cm
step_periode = 1.0  	# second
rate = 80.0    	# hz
walk_distance = 8.0 	# cm
swap_distance = 3.0 	# cm
rotate_angle = 15.0 	# degree

motion_state = 0    	#start_pose

dsp_ratio = 0.25
ssp_ratio = 0.5 - dsp_ratio
dsp_1_end = dsp_ratio * step_periode
ssp_1_end = dsp_1_end + ssp_ratio * step_periode
dsp_2_end = ssp_1_end + dsp_ratio * step_periode
ssp_2_end = dsp_2_end + ssp_ratio * step_periode


def walkSin(t, period, period_shift, mag, mag_shift):
    return (mag * math.sin(2 * math.pi / period * (t - period_shift)) + mag_shift)


def walk():
    T = np.linspace(0, step_periode, rate * step_periode)
    for t in T:
        dataPub = FullBodyIK()

        if (0 <= t < dsp_1_end):
            x = walkSin(t, dsp_ratio * step_periode * 2, 0 + dsp_ratio * step_periode/4, swap_distance/2, swap_distance/2)
            dataPub.right_leg.x = x
            dataPub.right_leg.z = feet_height
            dataPub.left_leg.x = -x
            dataPub.left_leg.z = feet_height

        if (dsp_1_end <= t < ssp_1_end):
            z = walkSin(t, ssp_ratio * step_periode, dsp_1_end + dsp_ratio * step_periode/4, walk_height/2, walk_height/2)
            y = (t - dsp_1_end) / (ssp_ratio * step_periode) * walk_distance
            dataPub.right_leg.x = swap_distance
            dataPub.right_leg.y = y
            dataPub.right_leg.z = feet_height - z
            dataPub.left_leg.x = -swap_distance
            dataPub.left_leg.z = feet_height

        if (ssp_1_end <= t < dsp_2_end):
            x = walkSin(t, dsp_ratio * step_periode *2, ssp_1_end + dsp_ratio * step_periode/4, swap_distance/2, swap_distance/2)
            dataPub.right_leg.x = -x
            dataPub.right_leg.z = feet_height
            dataPub.left_leg.x = x
            dataPub.left_leg.z = feet_height

        if (dsp_2_end <= t <= ssp_2_end):
            z = walkSin(t, ssp_ratio * step_periode, dsp_2_end + dsp_ratio * step_periode/4, walk_height/2, walk_height/2)
            y = (t - dsp_2_end) / (ssp_ratio * step_periode) * walk_distance
            dataPub.right_leg.x = -swap_distance
            dataPub.right_leg.z = feet_height
            dataPub.left_leg.x = swap_distance
            dataPub.left_leg.y = y
            dataPub.left_leg.z = feet_height - z

        fullBodyIK_pub.publish(dataPub)
        sleep(1/rate)

def turnRight():
    T = np.linspace(0, step_periode, rate * step_periode)
    for t in T:
        dataPub = FullBodyIK()

        if (0 <= t < dsp_1_end):
            x = walkSin(t, dsp_ratio * step_periode * 2, 0 + dsp_ratio * step_periode/4, swap_distance/2, swap_distance/2)
            dataPub.right_leg.x = x
            dataPub.right_leg.z = feet_height
            dataPub.left_leg.x = -x
            dataPub.left_leg.z = feet_height

        if (dsp_1_end <= t < ssp_1_end):
            z = walkSin(t, ssp_ratio * step_periode, dsp_1_end + dsp_ratio * step_periode/4, walk_height/2, walk_height/2)
            y = (t - dsp_1_end) / (ssp_ratio * step_periode) * walk_distance
            rotate = (t - dsp_1_end)/ (ssp_ratio * step_periode) * rotate_angle
            dataPub.right_leg.x = swap_distance
            dataPub.right_leg.y = y
            dataPub.right_leg.z = feet_height - z
            dataPub.right_leg.rotate = rotate
            dataPub.left_leg.x = -swap_distance
            dataPub.left_leg.z = feet_height

        if (ssp_1_end <= t < dsp_2_end):
            x = walkSin(t, dsp_ratio * step_periode * 2, ssp_1_end + dsp_ratio * step_periode/4, swap_distance/2, swap_distance/2)
            dataPub.right_leg.x = -x
            dataPub.right_leg.z = feet_height
            dataPub.right_leg.rotate = rotate_angle
            dataPub.left_leg.x = x
            dataPub.left_leg.z = feet_height

        if (dsp_2_end <= t <= ssp_2_end):
            z = walkSin(t, ssp_ratio * step_periode, dsp_2_end + dsp_ratio * step_periode/4, walk_height/2, walk_height/2)
            y = (t - dsp_2_end) / (ssp_ratio * step_periode) * walk_distance
            rotate = (t - dsp_2_end)/ (ssp_ratio * step_periode) * rotate_angle
            dataPub.right_leg.x = -swap_distance
            dataPub.right_leg.z = feet_height
            dataPub.right_leg.rotate = -rotate
            dataPub.left_leg.x = swap_distance
            dataPub.left_leg.y = y
            dataPub.left_leg.z = feet_height - z

        fullBodyIK_pub.publish(dataPub)
        sleep(1/rate)

def turnLeft():
    T = np.linspace(0, step_periode, rate * step_periode)
    for t in T:
        dataPub = FullBodyIK()

        if (0 <= t < dsp_1_end):
            x = walkSin(t, dsp_ratio * step_periode * 2, 0 + dsp_ratio * step_periode/4, swap_distance/2, swap_distance/2)
            dataPub.right_leg.x = -x
            dataPub.right_leg.z = feet_height
            dataPub.left_leg.x = x
            dataPub.left_leg.z = feet_height

        if (dsp_1_end <= t < ssp_1_end):
            z = walkSin(t, ssp_ratio * step_periode, dsp_1_end + dsp_ratio * step_periode/4, walk_height/2, walk_height/2)
            y = (t - dsp_1_end) / (ssp_ratio * step_periode) * walk_distance
            rotate = (t - dsp_1_end)/ (ssp_ratio * step_periode) * rotate_angle
            dataPub.right_leg.x = -swap_distance
            dataPub.right_leg.z = feet_height 
            dataPub.left_leg.x = swap_distance
            dataPub.left_leg.y = y
            dataPub.left_leg.z = feet_height - z
            dataPub.left_leg.rotate = rotate

        if (ssp_1_end <= t < dsp_2_end):
            x = walkSin(t, dsp_ratio * step_periode * 2, ssp_1_end + dsp_ratio * step_periode/4, swap_distance/2, swap_distance/2)
            dataPub.right_leg.x = x
            dataPub.right_leg.z = feet_height
            dataPub.left_leg.rotate = rotate_angle
            dataPub.left_leg.x = -x
            dataPub.left_leg.z = feet_height

        if (dsp_2_end <= t <= ssp_2_end):
            z = walkSin(t, ssp_ratio * step_periode, dsp_2_end + dsp_ratio * step_periode/4, walk_height/2, walk_height/2)
            y = (t - dsp_2_end) / (ssp_ratio * step_periode) * walk_distance
            rotate = (t - dsp_2_end)/ (ssp_ratio * step_periode) * rotate_angle
            dataPub.right_leg.x = swap_distance
            dataPub.right_leg.y = y
            dataPub.right_leg.z = feet_height - z
            dataPub.left_leg.rotate = -rotate
            dataPub.left_leg.x = -swap_distance
            dataPub.left_leg.z = feet_height - z

        fullBodyIK_pub.publish(dataPub)
        sleep(1/rate)

def steadyWalk():
    T = np.linspace(0, step_periode, rate * step_periode)
    for t in T:
        dataPub = FullBodyIK()

        if (0 <= t < dsp_1_end):
            x = walkSin(t, dsp_ratio * step_periode * 2, 0 + dsp_ratio * step_periode/4, swap_distance/2, swap_distance/2)
            dataPub.right_leg.x = x
            dataPub.right_leg.z = feet_height
            dataPub.left_leg.x = -x
            dataPub.left_leg.z = feet_height

        if (dsp_1_end <= t < ssp_1_end):
            z = walkSin(t, ssp_ratio * step_periode, dsp_1_end + dsp_ratio * step_periode/4, walk_height/2, walk_height/2)
            dataPub.right_leg.x = swap_distance
            dataPub.right_leg.z = feet_height - z
            dataPub.left_leg.x = -swap_distance
            dataPub.left_leg.z = feet_height

        if (ssp_1_end <= t < dsp_2_end):
            x = walkSin(t, dsp_ratio * step_periode * 2, ssp_1_end + dsp_ratio * step_periode/4, swap_distance/2, swap_distance/2)
            dataPub.right_leg.x = -x
            dataPub.right_leg.z = feet_height
            dataPub.left_leg.x = x
            dataPub.left_leg.z = feet_height

        if (dsp_2_end <= t <= ssp_2_end):
            z = walkSin(t, ssp_ratio * step_periode, dsp_2_end + dsp_ratio * step_periode/4, walk_height/2, walk_height/2)
            dataPub.right_leg.x = -swap_distance
            dataPub.right_leg.z = feet_height
            dataPub.left_leg.x = swap_distance
            dataPub.left_leg.z = feet_height - z

        fullBodyIK_pub.publish(dataPub)
        sleep(1/rate)


def startPose():
    dataPub = FullBodyIK()
    dataPub.right_leg.z = feet_height
    dataPub.left_leg.z = feet_height
    fullBodyIK_pub.publish(dataPub)
    sleep(1/rate)
    

def setMotionHandler(data):
    global motion_state
    motion_state = data.data
    rospy.loginfo('motion: ' + str(data.data))


def setWalkParamHandler(data):
    global walk_height, walk_distance, step_periode, rotate_angle
    global ssp_ratio, r_dsp_start, dsp_1_end, r_ssp_start, ssp_1_end, l_dsp_start, dsp_2_end, l_ssp_start, ssp_2_end
    
    walk_distance = data.step_distance
    walk_height = data.step_height
    step_periode = data.step_periode
    swap_distance = data.swap_distance
    rotate_angle = data.turn_angle

    dsp_1_end = dsp_ratio * step_periode
    ssp_1_end = dsp_1_end + ssp_ratio * step_periode
    dsp_2_end = ssp_1_end + dsp_ratio * step_periode
    ssp_2_end = dsp_2_end + ssp_ratio * step_periode


if __name__ == '__main__':
    rospy.Subscriber('set_motion', UInt8, setMotionHandler)
    rospy.Subscriber('set_walkParam', walkParam, setWalkParamHandler)

    while not rospy.is_shutdown():
        if motion_state == 0:
            startPose()
        elif motion_state == 1:
            steadyWalk()
        elif motion_state == 2:
            walk()
        elif motion_state == 3:
            turnRight()
        elif motion_state == 4:
            turnLeft()
