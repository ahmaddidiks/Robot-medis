#!/usr/bin/env python3

import rospy, rospkg, struct
from dynamixel_sdk import *
from std_msgs.msg import Bool
from configparser import SafeConfigParser
from dynamixel_control.msg import DynamixelPosList

ADDR_PRO_TORQUE_ENABLE1      = 24
ADDR_PRO_GOAL_POSITION1      = 30
ADDR_PRO_PRESENT_POSITION1   = 36

LEN_PRO_GOAL_POSITION1       = 2
LEN_PRO_PRESENT_POSITION1    = 2
LEN_PRO_TORQUE_ENABLE1    = 2
DXL_ID1 = 5

ADDR_PRO_TORQUE_ENABLE2      = 64
ADDR_PRO_GOAL_POSITION2      = 116
ADDR_PRO_PRESENT_POSITION2   = 132

LEN_PRO_GOAL_POSITION2       = 4
LEN_PRO_PRESENT_POSITION2    = 4
LEN_PRO_TORQUE_ENABLE2    = 4

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

rospy.init_node('dynamixel_control_node')

BAUDRATE = rospy.get_param("~dynamixel_baudrate", 1000000)
DEVICENAME = rospy.get_param("~dynamixel_device", "/dev/ttyACM0")
PROTOCOL_VERSION1 = 1#rospy.get_param("~dynamixel_protocol", 1.0)
PROTOCOL_VERSION2 = rospy.get_param("~dynamixel_protocol", 2.0)

portHandler = PortHandler(DEVICENAME)
packetHandler1 = PacketHandler(PROTOCOL_VERSION1)
packetHandler2 = PacketHandler(PROTOCOL_VERSION2)
PosSyncWrite1 = GroupSyncWrite(portHandler, packetHandler1, ADDR_PRO_GOAL_POSITION1, LEN_PRO_GOAL_POSITION1)
PosSyncWrite2 = GroupSyncWrite(portHandler, packetHandler2, ADDR_PRO_GOAL_POSITION2, LEN_PRO_GOAL_POSITION2)
TorqueSyncWrite1 = GroupSyncWrite(portHandler, packetHandler1, ADDR_PRO_TORQUE_ENABLE1, LEN_PRO_TORQUE_ENABLE1)
TorqueSyncWrite2 = GroupSyncWrite(portHandler, packetHandler2, ADDR_PRO_TORQUE_ENABLE2, LEN_PRO_TORQUE_ENABLE2)

configFile = rospkg.RosPack().get_path('dynamixel_control')+"/config/dynamixel.config"
config = SafeConfigParser()


def init():
    #PROTOCOL 1
    dxl_comm_result, dxl_error = packetHandler1.write1Bytedxl_comm_result, dxl_error = packetHandler1.write1ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_TORQUE_ENABLE1, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logwarn(f"[ID:{DXL_ID1}] {packetHandler1.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        rospy.logwarn(f"[ID:{DXL_ID1}] {packetHandler1.getRxPacketError(dxl_error)}")
    
    #PROTOCOL 2
    for DXL_ID2 in range(1,5):
        dxl_comm_result, dxl_error = packetHandler2.write1Bytedxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_TORQUE_ENABLE2, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logwarn(f"[ID:{DXL_ID2}] {packetHandler2.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            rospy.logwarn(f"[ID:{DXL_ID2}] {packetHandler2.getRxPacketError(dxl_error)}")



def enTorqueHandler(data):
    #PROTOCOL 1
    dxl_comm_result, dxl_error = packetHandler1.write1ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_TORQUE_ENABLE1, int(data.data))
    if dxl_comm_result != COMM_SUCCESS:
        rospy.logwarn(f"[ID:{DXL_ID1}] {packetHandler1.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        rospy.logwarn(f"[ID:{DXL_ID1}] {packetHandler1.getRxPacketError(dxl_error)}")
    
    #PROTOCOL 2
    for DXL_ID2 in range(1, 5):
        dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_TORQUE_ENABLE2, int(data.data))
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logwarn(f"[ID:{DXL_ID2}] {packetHandler2.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            rospy.logwarn(f"[ID:{DXL_ID2}] {packetHandler2.getRxPacketError(dxl_error)}")

def setPositionHandler(data):

    for dxl in data.dynamixel:
        #DYNAMIXEL AX12 PROTOCOL1
        if dxl.id == 5:
            #limit gripper deg = 160
            if dxl.position > 160:
                dxl.position = 160
            else:
                dxl.position = dxl.position
            pwm = int(dxl.position * 1023.0 / 300.0)
            dxl_comm_result, dxl_error = packetHandler1.write4ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_GOAL_POSITION1, pwm)
            if dxl_comm_result != COMM_SUCCESS:
                print(f"{packetHandler1.getTxRxResult(dxl_comm_result)}")
            elif dxl_error != 0:
                print(f"{packetHandler1.getRxPacketError(dxl_error)}")
        
        #DYNAMIXEL MX AND X SERIES PROTargs="-d $(find urdf_manipulator)/urdf.rviz" />OCOL2 
        else:
            pwm = dxl.position * 4095.0 / 360.0 
            data_send = list(struct.unpack('4B', struct.pack("I", limitCheck(dxl.id, int(pwm)))))
            dxl_addparam_result = PosSyncWrite2.addParam(dxl.id, data_send)
            if dxl_addparam_result != True:
                rospy.logerr(f"[ID:{dxl.id}] GroupSyncWrite addparam failed")

    # dxl_comm_result1 = PosSyncWrite1.txPacket()
    dxl_comm_result2 = PosSyncWrite2.txPacket()
    # if dxl_comm_result1 != COMM_SUCCESS:
    #     rospy.logerr(f"{packetHandler1.getTxRxResult(dxl_comm_result1)}" )
    if dxl_comm_result2 != COMM_SUCCESS:
        rospy.logerr(f"{packetHandler2.getTxRxResult(dxl_comm_result2)}" )

    # PosSyncWrite1.clearParam()
    PosSyncWrite2.clearParam()

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
    #PROTOCOL1
    dxl_comm_result, dxl_error = packetHandler1.write1ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_TORQUE_ENABLE1, TORQUE_DISABLE)
    #PROTOCOL2
    for DXL_ID2 in range(1, 5):
        dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_TORQUE_ENABLE2, TORQUE_DISABLE)

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
    # rospy.Subscriber('en_torque', Bool, enTorqueHandler)
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