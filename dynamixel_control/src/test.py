# real file

#!/usr/bin/env python

import rospy, rospkg, struct
from dynamixel_sdk import *
from std_msgs.msg import Bool
from ConfigParser import SafeConfigParser
from dynamixel_control.msg import DynamixelPosList

ADDR_PRO_TORQUE_ENABLE      = 64
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4
LEN_PRO_TORQUE_ENABLE    = 4

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

rospy.init_node('dynamixel_control_node')

BAUDRATE = rospy.get_param("~dynamixel_baudrate", 2000000)
DEVICENAME = rospy.get_param("~dynamixel_device", "/dev/ttyUSB0")
PROTOCOL_VERSION = rospy.get_param("~dynamixel_protocol", 2.0)

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
PosSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
TorqueSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_TORQUE_ENABLE, LEN_PRO_TORQUE_ENABLE)

configFile = rospkg.RosPack().get_path('dynamixel_control')+"/config/dynamixel.config"
config = SafeConfigParser()


def init():
    for DXL_ID in range(1, 20):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logwarn("[ID:%d] %s" % (DXL_ID, packetHandler.getTxRxResult(dxl_comm_result)))
        elif dxl_error != 0:
            rospy.logwarn("[ID:%d] %s" % (DXL_ID, packetHandler.getRxPacketError(dxl_error)))  


def enTorqueHandler(data):
    for DXL_ID in range(1, 20):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, int(data.data))
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logwarn("[ID:%d] %s" % (DXL_ID, packetHandler.getTxRxResult(dxl_comm_result)))
        elif dxl_error != 0:
            rospy.logwarn("[ID:%d] %s" % (DXL_ID, packetHandler.getRxPacketError(dxl_error)))


def setPositionHandler(data):
    for dxl in data.dynamixel:
        pwm = dxl.position * 4095 / 360
        data_send = list(struct.unpack('4B', struct.pack("I", limitCheck(dxl.id, pwm))))
        dxl_addparam_result = PosSyncWrite.addParam(dxl.id, data_send)
        if dxl_addparam_result != True:
            rospy.logerr("[ID:%03d] GroupSyncWrite addparam failed" % dxl.id)

    dxl_comm_result = PosSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    PosSyncWrite.clearParam()

def limitCheck(id, pwm):
    min = int(config.get("id_"+str(id), "min"))
    max = int(config.get("id_"+str(id), "max"))
    if pwm < min:
        return min
    elif pwm > max:
        return max
    else:
        return pwm
    

def onShutdown():
    for DXL_ID in range(1, 20):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)

    portHandler.closePort()


def loop():
    pass


if __name__ == '__main__':
    if portHandler.openPort():
        rospy.loginfo("Succeeded to open the port")
    else:
        rospy.logerr("Failed to open the port")
        while True:
            pass

    if portHandler.setBaudRate(BAUDRATE):
        rospy.loginfo("Succeeded to change the baudrate")
    else:
        rospy.logerr("Failed to change the baudrate")
        while True:
            pass

    rospy.Subscriber('set_position', DynamixelPosList, setPositionHandler)
    rospy.Subscriber('en_torque', Bool, enTorqueHandler)
    rospy.on_shutdown(onShutdown)

    config.read(configFile)
    init()

    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            loop()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


# alternative file

#!/usr/bin/env python

import rospy, rospkg, struct
from dynamixel_sdk import *
from std_msgs.msg import Bool, UInt16
from ConfigParser import SafeConfigParser
from dynamixel_control.msg import DynamixelPosList

ADDR_PRO_TORQUE_ENABLE      = 64
ADDR_PRO_VELOCITY           = 112
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_VELOCITY            = 4
LEN_PRO_PRESENT_POSITION    = 4
LEN_PRO_TORQUE_ENABLE    = 4

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

position = False
velocity = False

rospy.init_node('dynamixel_control_node')

BAUDRATE = rospy.get_param("~dynamixel_baudrate", 2000000)
DEVICENAME = rospy.get_param("~dynamixel_device", "/dev/ttyUSB0")
PROTOCOL_VERSION = rospy.get_param("~dynamixel_protocol", 2.0)

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
# BulkWrite = GroupBulkWrite(portHandler, packetHandler)
PosSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
VelSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_VELOCITY, LEN_PRO_VELOCITY)
TorqueSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_TORQUE_ENABLE, LEN_PRO_TORQUE_ENABLE)

configFile = rospkg.RosPack().get_path('dynamixel_control')+"/config/dynamixel.config"
config = SafeConfigParser()


def init():
    for DXL_ID in range(1, 20):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logwarn("[ID:%d] %s" % (DXL_ID, packetHandler.getTxRxResult(dxl_comm_result)))
        elif dxl_error != 0:
            rospy.logwarn("[ID:%d] %s" % (DXL_ID, packetHandler.getRxPacketError(dxl_error)))  


def enTorqueHandler(data):
    for DXL_ID in range(1, 20):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, int(data.data))
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logwarn("[ID:%d] %s" % (DXL_ID, packetHandler.getTxRxResult(dxl_comm_result)))
        elif dxl_error != 0:
            rospy.logwarn("[ID:%d] %s" % (DXL_ID, packetHandler.getRxPacketError(dxl_error)))


def setPositionHandler(data):
    for dxl in data.dynamixel:
        pwm = dxl.position * 4095 / 360
        data_send = list(struct.unpack('4B', struct.pack("I", limitCheck(dxl.id, pwm))))
        dxl_addparam_result = PosSyncWrite.addParam(dxl.id, data_send)
        if dxl_addparam_result != True:
            rospy.logerr("[ID:%03d] GroupSyncWrite addparam failed" % dxl.id)

    dxl_comm_result = PosSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    PosSyncWrite.clearParam()

def setVelocityHandler(data):
    for dxl in data.dynamixel:
        data_send = list(struct.unpack('4B', struct.pack("I", int(data.data))))
        dxl_addparam_result = VelSyncWrite.addParam(dxl.id, data_send)
        if dxl_addparam_result != True:
            rospy.logerr("[ID:%03d] GroupSyncWrite addparam failed" % dxl.id)

    dxl_comm_result = VelSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logerr("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    VelSyncWrite.clearParam()

def limitCheck(id, pwm):
    min = int(config.get("id_"+str(id), "min"))
    max = int(config.get("id_"+str(id), "max"))
    if pwm < min:
        return min
    elif pwm > max:
        return max
    else:
        return pwm
    

def onShutdown():
    for DXL_ID in range(1, 20):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)

    portHandler.closePort()


def loop():
    pass


if __name__ == '__main__':
    if portHandler.openPort():
        rospy.loginfo("Succeeded to open the port")
    else:
        rospy.logerr("Failed to open the port")
        while True:
            pass

    if portHandler.setBaudRate(BAUDRATE):
        rospy.loginfo("Succeeded to change the baudrate")
    else:
        rospy.logerr("Failed to change the baudrate")
        while True:
            pass

    rospy.Subscriber('set_position', DynamixelPosList, setPositionHandler)
    rospy.Subscriber('set_velocity', UInt16, setVelocityHandler)
    rospy.Subscriber('en_torque', Bool, enTorqueHandler)
    rospy.on_shutdown(onShutdown)

    config.read(configFile)
    init()

    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            loop()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
