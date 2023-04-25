from dynamixel_sdk import *
import sys, tty, termios
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from collections import deque
import numpy as np
import cv2
import pyrealsense2 as rs
import time
from ultralytics import YOLO

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

    def increment_array(self, start_array, end_array, time_period):
        rate_of_change = [(end_array[i] - start_array[i]) / time_period for i in range(len(start_array))]
        current_array = start_array.copy()
        increment_time = time_period / 5
        for i in range(5):
            current_array = [current_array[j] + rate_of_change[j] * increment_time for j in range(len(current_array))]
            print(current_array)
            time.sleep(increment_time)
            target_angle = [angle_to_pos(i+180) for i in open_maipulator.inverse_kinematics(current_array)*180/np.pi]
            for i in range(1, 4):      
                self.move_to_goal(target_angle)
        return end_array

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

def ZOH(goal_arr, target_angle, target_angle_stack):
    target_angle_stack.append(target_angle)
    if len(target_angle_stack) > 5:
        target_angle_stack.popleft()

    for i in range(1, 4):
        if abs(goal_arr[i] - target_angle[i]) > 400:
            print('joint_limit')
            return target_angle_stack[0]
    return target_angle


def realsense_on():
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    pipeline.start(config)

    return pipeline

def get_realsense_color_depth_frame(pipeline):
    frames = pipeline.wait_for_frames()
    
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    color_image = np.asanyarray(color_frame.get_data())

    return color_image, depth_frame

def predict_result(model, input_image):
    predicted_results = model(input_image, imgsz=640, conf = 0.7)[0]

    detect_name = predicted_results.names

    detect_ob_num = predicted_results.boxes.cls.tolist()
    detect_ob_percentage = predicted_results.boxes.conf.tolist()
    detect_ob_cordinate = predicted_results.boxes.xyxy.tolist()

    return detect_ob_num, detect_ob_percentage, detect_ob_cordinate, detect_name
# detect_ob_num, detect_ob_percentage, detect_ob_cordinate, detect_name 
def draw_detect_object(input_image, x1, y1, x2, y2, detect_name, detect_ob_num, detect_ob_percentage):
    goal_point = (480,320)
    cv2.circle(input_image, goal_point, radius=5, color=(0, 0, 255), thickness=-1)
    cv2.circle(input_image, (int((x1+x2)/2), int((y1+y2)/2)), radius=5, color=(255, 0, 0), thickness=-1)

    cv2.line(input_image, goal_point, (int((x1+x2)/2), int((y1+y2)/2)), color=(0, 0, 255), thickness=2)

    # detecting_object_text
    cv2.putText(input_image,  detect_name[detect_ob_num[0]] + " " + str(round(detect_ob_percentage[0]*100, 2)) + '%',
        (x1, y1-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_4) 
    
    # detecting_object
    cv2.rectangle(input_image, (x1, y1), (x2, y2), (0,255,0), 3)
    print('x : ', goal_point[0] - int((x1+x2)/2), 'y : ', goal_point[1]- int((y1+y2)/2))
    return (int((x1+x2)/2), int((y1+y2)/2))

def camera_off(pipeline):
    pipeline.stop()
    # cap.release()
    cv2.destroyAllWindows()


def main():
    target_vector = [0.01399388  , 0.0, 0.00770011]
    dxl_goal_position = [angle_to_pos(i+180) for i in open_maipulator.inverse_kinematics(target_vector)*180/np.pi]
    dxl_goal_position[0], dxl_goal_position[4] = angle_to_pos(180), angle_to_pos(100)
    pipeline = realsense_on()
    model = YOLO('rsc/best_yolo8v_trash_x.pt')
    mid_point_list = [deque([]) for _ in range(2)]
    average_step_size = 5

    dynamixel = DynamixelControl([11, 12, 13, 14, 15])
    dynamixel.open_port_and_baud()
    grab_flag = False
    goal_point = (420,240) 
    while True:
        print(open_maipulator.forward_kinematics([(pos_to_angle(dxl_goal_position[i])-180)*np.pi/180 for i in range(5)]))
        color_image, _ = get_realsense_color_depth_frame(pipeline)
        frame_height, frame_width, _ = color_image.shape
        frame_mid_point = (frame_height//2, frame_width//2)

        # predict
        detect_ob_num, detect_ob_percentage, detect_ob_cordinate, detect_name = predict_result(model, color_image)

        if detect_ob_cordinate:
            x1, y1, x2, y2 = map(int, detect_ob_cordinate[0])

            mid_point_list[0].append(frame_mid_point[0] - int((x1+x2)/2))
            mid_point_list[1].append(frame_mid_point[1] - int((y1+y2)/2))
            # pop the first element if it more than step_size
            if len(mid_point_list[0]) > average_step_size:
                mid_point_list[0].popleft()
                mid_point_list[1].popleft()



        cv2.imshow('Color frame', color_image)

        key_input = cv2.waitKey(1)
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
            dxl_goal_position[0] += angle_to_pos(1)
        elif key_input == ord('z'):
            dxl_goal_position[0] -= angle_to_pos(1)
        elif key_input == ord('a'):
            target_vector[0] += 0.001
        elif key_input == ord('d'):
            target_vector[0] -= 0.001
        elif key_input == ord('w'):
            target_vector[2] += 0.001
        elif key_input == ord('s'):
            target_vector[2] -= 0.001
        elif key_input == ord('r'):
            target_vector = [0.01399388  , 0.0, 0.00770011]
            dxl_goal_position[0] = angle_to_pos(180)
        
        # dumping plastic
        elif key_input == ord('p'):
            dxl_goal_position[0] = angle_to_pos(180+90)
            dynamixel.move_to_goal(dxl_goal_position)

            # dynamixel.increment_array(target_vector, [0.0268435  , 0.0, 0.01415466], 5)
            target_vector = [0.030716  , 0.0, 0.023171]
            target_angle = [angle_to_pos(i+180) for i in open_maipulator.inverse_kinematics(target_vector)*180/np.pi]
            dynamixel.move_to_goal(dxl_goal_position)
            dxl_goal_position[4] = angle_to_pos(100)
            dynamixel.move_to_goal(dxl_goal_position)
            

        target_angle = [angle_to_pos(i+180) for i in open_maipulator.inverse_kinematics(target_vector)*180/np.pi]
        
        for i in range(1, 4):
            dxl_goal_position[i] = target_angle[i]        
        dynamixel.move_to_goal(dxl_goal_position)
        

    dynamixel.kill_process()
    camera_off(pipeline)


if __name__ == '__main__':
    main()