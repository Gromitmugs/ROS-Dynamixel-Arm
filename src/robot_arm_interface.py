#!/usr/bin/env python

import os
import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
import ctypes

# Variables
PROTOCOL_VERSION            = 2.0
DEVICENAME                  = '/dev/ttyACM0'

# Control Table Addresses
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_PROFILE_VELOCITY = 112

BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
ADDR_LED_RED                = 65
LEN_LED_RED                 = 1         # Data Byte Length
LEN_GOAL_POSITION           = 4         # Data Byte Length
LEN_PRESENT_POSITION        = 4         # Data Byte Length
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual

# Robot Arm IDs
DXL_BASE = 1
DXL_SHOULDER_2 = 2
DXL_SHOULDER_3 = 3
DXL_ELBOW = 4
DXL_WRIST = 5
DXL_GRIPPER = 6

DXL_JOINTS = [DXL_BASE, DXL_SHOULDER_2, DXL_SHOULDER_3, DXL_ELBOW, DXL_WRIST, DXL_GRIPPER]
DEFAULT_PROFILE_VELOCITY = 20 # value * 0.229 rpm

# Setup
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

def initialize_dxls(DXL_JOINTS):
    for dxl_id in DXL_JOINTS:
        enable_torque(dxl_id)
        set_profile_velocity(dxl_id, DEFAULT_PROFILE_VELOCITY)

    #add param test
        dxl_addparam_result = groupBulkRead.addParam(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % dxl_id)
            quit()

def enable_torque(dxl_id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d torque enabled" % dxl_id)

def disable_torque(dxl_id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d torque disabled" % dxl_id)

def set_profile_velocity(dxl_id, vel_raw):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, vel_raw)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d speed set to %d" % (dxl_id, vel_raw))

def set_param_goal_position(position):
    return [DXL_LOBYTE(DXL_LOWORD(position)),
            DXL_HIBYTE(DXL_LOWORD(position)),
            DXL_LOBYTE(DXL_HIWORD(position)),
            DXL_HIBYTE(DXL_HIWORD(position))]


def get_arm_position(req):
    arm_position = [-1,-1,-1,-1,-1,-1]

    # dxl_comm_result = groupBulkRead.txRxPacket()
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # for dxl_id in DXL_JOINTS:
    #     dxl_getdata_result = groupBulkRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    #     if dxl_getdata_result != True:
    #         print("[ID:%03d] groupBulkRead getdata failed" % dxl_id)
    #     else:
    #         dxl_present_position = groupBulkRead.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    #         arm_position[dxl_id-1] = dxl_present_position

    for dxl_id in DXL_JOINTS:
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            arm_position[dxl_id-1] = dxl_present_position

    return arm_position[0], arm_position[1], arm_position[2], arm_position[3], arm_position[4], arm_position[5]

def set_arm_position(req):
    groupBulkWrite.clearParam()
    arm_positions = [req.position_1, req.position_2, req.position_3, req.position_4, req.position_5, req.position_6]

    for position in arm_positions:
        if not (-1 <= position <= 4095):
            return False
        print(position)

    for position, dxl_id in zip(arm_positions,DXL_JOINTS):
        if position == -1:
            continue
        print(position)
        param_goal_position = set_param_goal_position(position)
        dxl_addparam_result = groupBulkWrite.addParam(dxl_id, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, param_goal_position)
        print("ID%d: position=%d" % (dxl_id, position))
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % dxl_id)

    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return False
    return True

def set_torque(req):
    torque_settings = [req.torque_1, req.torque_2, req.torque_3, req.torque_4, req.torque_5, req.torque_6]
    for val in torque_settings:
        if not (0<=val<=1):
            return False

    for val, dxl_id in zip(torque_settings, DXL_JOINTS):
        if val == TORQUE_DISABLE:
            disable_torque(dxl_id)
        elif val == TORQUE_ENABLE:
            enable_torque(dxl_id)
    return True

def read_write_py_node():
    rospy.init_node('read_write_py_node')
    rospy.Service('get_arm_position', GetArmPosition, get_arm_position)
    rospy.Service('set_arm_position', SetArmPosition, set_arm_position)
    rospy.Service('set_torque', SetTorque, set_torque)
    rospy.spin()

def main():
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        quit()

    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        quit()

    initialize_dxls(DXL_JOINTS)

    print("Ready to get & set Position.")
    read_write_py_node()


if __name__ == '__main__':
    main()