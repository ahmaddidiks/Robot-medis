#!/usr/bin/env python
import rospy, rospkg, struct
from dynamixel_sdk import *
from std_msgs.msg import Int16, Bool
from ConfigParser import SafeConfigParser
from dynamixel_control.msg import DynamixelPos, DynamixelPosList, DynamixelState, DynamixelStateList, DynamixelTorqueEnable

ADDR_PRO_TORQUE_ENABLE      = 64
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4
LEN_PRO_TORQUE_ENABLE    = 4

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 20

rospy.init_node('dynamixel_control_node')

BAUDRATE = rospy.get_param("~dynamixel_baudrate", 2000000)
DEVICENAME = rospy.get_param("~dynamixel_device", "/dev/ttyUSB0")
PROTOCOL_VERSION = rospy.get_param("~dynamixel_protocol", 2.0)

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
PosSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
TorqueSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_TORQUE_ENABLE, LEN_PRO_TORQUE_ENABLE)

#present_position_pub = rospy.Publisher('present_position', Int32, queue_size=1)

configFile = rospkg.RosPack().get_path('dynamixel_control')+"/config/dynamixel.config"
config = SafeConfigParser()


def enTorqueHandler(data):
    if (data.torque):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, data.id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    else:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, data.id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    errorCheck(dxl_comm_result, dxl_error)


def setPositionHandler(data):
    pwm = data.position * 4095 / 360
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, data.id, ADDR_PRO_GOAL_POSITION, pwm)
    errorCheck(dxl_comm_result, dxl_error)


def setPositionSyncHandler(data):
    for dxl in data.dynamixel:
        pwm = dxl.position * 4095 / 360
        data_send = list(struct.unpack('4B', struct.pack("I", pwm)))
        dxl_addparam_result = PosSyncWrite.addParam(dxl.id, data_send)
        if dxl_addparam_result != True:
            rospy.logerr("[ID:%03d] GroupSyncWrite addparam failed" % DXL_ID)

    dxl_comm_result = PosSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    PosSyncWrite.clearParam()    


def errorCheck(dxl_comm_result, dxl_error):
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logwarn("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.logwarn("%s" % packetHandler.getRxPacketError(dxl_error))  
    

def onShutdown():
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    #errorCheck(dxl_comm_result, dxl_error)

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

    rospy.Subscriber('set_position', DynamixelPos, setPositionHandler)
    rospy.Subscriber('set_position_sync', DynamixelPosList, setPositionSyncHandler)
    rospy.Subscriber('en_torque', DynamixelTorqueEnable, enTorqueHandler)
    rospy.on_shutdown(onShutdown)

    config.read(configFile)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        #dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
        #errorCheck(dxl_comm_result, dxl_error)
        #present_position_pub.publish(dxl_present_position)
        
        rate.sleep()

    rospy.spin()
    
