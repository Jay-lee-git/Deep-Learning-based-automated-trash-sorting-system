from dynamixel_sdk import *
import sys, tty, termios
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from collections import deque
import numpy as np

open_maipulator = Chain(name='test_arm', links=[
    OriginLink(),
    URDFLink(
      
      name="link0",
      # origin_translation=[0.012, 0, 0.017],
      origin_translation=[0, 0, 0.003],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      use_symbolic_matrix = False,
      joint_type = "revolute"
    ),    
    URDFLink(
      name="link1",
      # origin_translation=[0.012, 0, 0.017],
      origin_translation=[0, 0, 0.013],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      use_symbolic_matrix = False,
      joint_type = "revolute"
    ),
    URDFLink(
      name="link2",
      # origin_translation=[0.012, 0, 0.017],
      origin_translation=[0.013, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      use_symbolic_matrix = False,
      joint_type = "revolute"
    ),    
    URDFLink(
      name="link3",
      # origin_translation=[0, 0, 0.0595],
      origin_translation=[0.013, 0, 0],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      use_symbolic_matrix = False,
      joint_type = "revolute"
    ),
])


class DynamixelControl:
    def __init__(self, DXL_ID_list):
        self.DEVICENUM = 5
        self.DXL_MOVING_STATUS_THRESHOLD = 60
        self.DEVICENAME = '/dev/ttyUSB0'
        self.DXL_ID_list = DXL_ID_list
        self.protocol_version = 2.0

        self.portHandler_list = [PortHandler(self.DEVICENAME) for _ in range(self.DEVICENUM)]
        self.packetHandler_list = [PacketHandler(self.protocol_version) for _ in range(self.DEVICENUM)]

    
    def open_port_and_baud(self):
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.ADDR_TORQUE_ENABLE = 64
        self.baudrate = 1000000
        for i in range(5):
            if self.portHandler_list[i].openPort():
                print("Succeeded to open the port | ", end="")
            else:
                print("Failed to open the port")
                
            if self.portHandler_list[i].setBaudRate(self.baudrate):
                print("Succeeded to change the baudrate | ", end="")
            else:
                print("Failed to change the baudrate")

            dxl_comm_result, dxl_error = self.packetHandler_list[i].write1ByteTxRx(self.portHandler_list[i], self.DXL_ID_list[i], self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler_list[i].getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler_list[i].getRxPacketError(dxl_error))
            else:
                print("Dynamixel has been successfully connected")

    def move_to_goal(self, dxl_goal_position):
        ADDR_GOAL_POSITION = 116
        for i in range(0, self.DEVICENUM):
            dxl_comm_result, dxl_error = self.packetHandler_list[i].write4ByteTxRx(self.portHandler_list[i], 
                                                                    self.DXL_ID_list[i], ADDR_GOAL_POSITION, dxl_goal_position[i])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler_list[0].getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler_list[0].getRxPacketError(dxl_error))
    
    def kill_process(self):
        for i in range(0, self.DEVICENUM):
            dxl_comm_result, dxl_error = self.packetHandler_list[i].write1ByteTxRx(self.portHandler_list[i], self.DXL_ID_list [i], self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            self.portHandler_list[i].closePort()


def angle_to_pos(angle):
    return int(4095/360 * angle)

def pos_to_angle(pos):
    return int(pos*360 /4095)
    
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def ZOH(arr1, arr2):
    for i in range(1, 4):
        if abs(arr1[i] - arr2[i]) > 400:
            print('joint_limit')
            return arr1
    return arr2


def main():
    # target_vector = [0.025  , 0., 0.02]
    target_vector = [0.01399388  , 0.0, 0.00770011]
    # dxl_goal_position = [angle_to_pos(180), angle_to_pos(160), angle_to_pos(167), angle_to_pos(167), angle_to_pos(50)]
    dxl_goal_position = [angle_to_pos(i+180) for i in open_maipulator.inverse_kinematics(target_vector)*180/np.pi]

    dynamixel = DynamixelControl([11, 12, 13, 14, 15])
    dynamixel.open_port_and_baud()
    grab_flag = False

    while True:
        print(open_maipulator.forward_kinematics([(pos_to_angle(dxl_goal_position[i])-180)*np.pi/180 for i in range(5)]))
        key_input = ord(getch())
        if key_input == 27:
            break
        elif key_input == ord('e'):
            if grab_flag:
                dxl_goal_position[4] = angle_to_pos(240)
                grab_flag = False
            else:
                dxl_goal_position[4] = angle_to_pos(100)
                grab_flag = True

        elif key_input == ord('c'):
            dxl_goal_position[0] += angle_to_pos(5)
        elif key_input == ord('z'):
            dxl_goal_position[0] -= angle_to_pos(5)
        elif key_input == ord('a'):
            target_vector[0] += 0.001
        elif key_input == ord('d'):
            target_vector[0] -= 0.001
        elif key_input == ord('w'):
            target_vector[2] += 0.001
        elif key_input == ord('s'):
            target_vector[2] -= 0.001
        elif key_input == ord('r'):
            dxl_goal_position = [angle_to_pos(180), angle_to_pos(160), angle_to_pos(167), angle_to_pos(270), angle_to_pos(160)]
        dynamixel.move_to_goal(dxl_goal_position)

        target_angle = [angle_to_pos(i+180) for i in open_maipulator.inverse_kinematics(target_vector)*180/np.pi]
        
        # dxl_goal_position = ZOH(dxl_goal_position, target_angle)
        for i in range(1,4):
            dxl_goal_position[i] = target_angle[i]

        test = []
        for i in range(1, 4):
            test.append(dxl_goal_position[i] - target_angle[i])
        print(dxl_goal_position , target_angle)
        print(test)

        
    dynamixel.kill_process()



if __name__ == '__main__':
    main()