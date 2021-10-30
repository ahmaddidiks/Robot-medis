#!/usr/bin/env python
import rospy
from dynamixel_sdk import *
from std_msgs.msg import Int32, Bool
from ConfigParser import SafeConfigParser

ADDR_PRO_TORQUE_ENABLE      = 64
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 20

rospy.init_node('dynamixel_control_node')

DXL_ID = rospy.get_param("~dynamixel_id", 1)
BAUDRATE = rospy.get_param("~dynamixel_baudrate", 57600)
DEVICENAME = rospy.get_param("~dynamixel_device", "/dev/ttyUSB0")

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(2.0)

present_position_pub = rospy.Publisher('present_position', Int32, queue_size=1)


def enTorqueHandler(data):
    if (data.data):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    else:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    errorCheck(dxl_comm_result, dxl_error)


def setPositionHandler(data):
	degree = data.data/360 * 4095
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, degree)
    errorCheck(dxl_comm_result, dxl_error)


def errorCheck(dxl_comm_result, dxl_error):
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logwarn("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.logwarn("%s" % packetHandler.getRxPacketError(dxl_error))  


def loadConfig():
    config = SafeConfigParser()
    global l_el_id, l_sho_pitch_id
    config.read(configFile)
    l_el_id = config.get('dynamixel_id', 'l_el')
    l_sho_pitch_id = config.get('dynamixel_id', 'l_sho_pitch')
    

def onShutdown():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    errorCheck(dxl_comm_result, dxl_error)

    portHandler.closePort()


if __name__ == '__main__':
    # Open port
    if portHandler.openPort():
        rospy.loginfo("Succeeded to open the port")
    else:
        rospy.logerr("Failed to open the port")
        while True:
            pass

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        rospy.loginfo("Succeeded to change the baudrate")
    else:
        rospy.logerr("Failed to change the baudrate")
        while True:
            pass  

    rospy.Subscriber('set_position', Int32, setPositionHandler)
    rospy.Subscriber('en_torque', Bool, enTorqueHandler)
    rospy.on_shutdown(onShutdown)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
        errorCheck(dxl_comm_result, dxl_error)
        present_position_pub.publish(dxl_present_position)
        
        rate.sleep()

    rospy.spin()
    
