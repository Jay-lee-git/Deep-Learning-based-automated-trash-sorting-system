from dynamixel_sdk import * 
import cv2
import numpy as np
import sys, tty, termios
from datetime import datetime 
import pyrealsense2 as rs
def pos_to_angle(pos):
    return int(pos/4095*365)

def angle_to_pos(angle):
    return int(4095/365 * angle)

# Open port baudrate
def open_port_and_baud(portHandler_list, index):
    # Open port
    if portHandler_list.openPort():        print(f"{index}-Succeeded to open the port | ", end=" ")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()
    # Set port baudrate
    if portHandler_list.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate | ", end=" ")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
def realsense_config():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    return pipeline
def get_realsense_frame(pipeline):
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    return color_image
# Enable Dynamixel Torque
def enable_torque(packetHandler, portHandler_list, DXL_ID):
    global ADDR_TORQUE_ENABLE, TORQUE_ENABLE, COMM_SUCCESS
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler_list, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel has been successfully connected")


def get_camera_goal_pos(x1,y1, x2,y2,frame):
    frame_height, frame_width, _ = frame.shape
    # print('frane width and height', frame_width, frame_height)
    frame_mid_point = (frame_width//2, frame_height//2)
    # print(frame_mid_point)
    target_mid_point = ((x1+x2)//2 -26, (y1+y2)//2 -2)
    return frame_mid_point, target_mid_point

if __name__ == '__main__':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product
    BAUDRATE                    = 1000000
    PROTOCOL_VERSION            = 2.0

    TARGET_INDEX_CODE = 14.03
    DXL_ID_list = [11, 12, 13, 14, 15]
    DEVICE_NUM = len(DXL_ID_list)
    DEVICENAME = '/dev/ttyUSB0'
    TORQUE_ENABLE               = 1     # Value for enabling the torque
    TORQUE_DISABLE              = 0     # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 60    # Dynamixel moving status threshold
    pipeline = realsense_config()
    index = 0
    target_pixel_threshold = 10
    # Goal position
    defalut_position = [angle_to_pos(180), angle_to_pos(100), angle_to_pos(250), angle_to_pos(180), angle_to_pos(180)]
    dxl_goal_position = defalut_position
    portHandler_list = [PortHandler(DEVICENAME) for _ in range(DEVICE_NUM)]
    packetHandler_list = [PacketHandler(PROTOCOL_VERSION) for _ in range(DEVICE_NUM)]

    for i in range(0, DEVICE_NUM):
        open_port_and_baud(portHandler_list[i], i)
        enable_torque(packetHandler_list[i], portHandler_list[i], DXL_ID_list[i])



    while True:
        frame = get_realsense_frame(pipeline)
        # Write goal position
        for i in range(0, DEVICE_NUM):
            dxl_comm_result, dxl_error = packetHandler_list[i].write4ByteTxRx(portHandler_list[i], 
                                                                    DXL_ID_list[i], ADDR_GOAL_POSITION, dxl_goal_position[i])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler_list[0].getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler_list[0].getRxPacketError(dxl_error))
        

        key_input = cv2.waitKey(1)
        if key_input == 27:
            break
        elif key_input == ord('w'):
            dxl_goal_position[3] -= angle_to_pos(1)
        elif key_input == ord('s'):
            dxl_goal_position[3] += angle_to_pos(1)
        elif key_input == ord('a'):
            dxl_goal_position[0] += angle_to_pos(1)
        elif key_input == ord('d'):
            dxl_goal_position[0] -= angle_to_pos(1)
        elif key_input == ord('l'):
            laser_flag = True
        # out.write(frame)
        cv2.imshow('frame', frame)

        for i in range(0, DEVICE_NUM):
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler_list[i].read4ByteTxRx(portHandler_list[i], DXL_ID_list[i], ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:     
                print("%s" % packetHandler_list[i].getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler_list[i].getRxPacketError(dxl_error))
           
    # Disable Dynamixel Torque
    for i in range(0, DEVICE_NUM):
        dxl_comm_result, dxl_error = packetHandler_list[i].write1ByteTxRx(portHandler_list[i], DXL_ID_list [i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    # Close port
    portHandler_list[0].closePort()
    # cap.release()
    # out.release()
