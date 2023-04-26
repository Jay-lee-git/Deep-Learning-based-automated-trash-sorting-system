

# Deep-Learning-based-automated-trash-sorting-system

automated trash sorting system using YOLOV8x  
can categrize 4 trash type  
- can, paper, glass_bottle, pet


<!-- ![sorting gif](https://user-images.githubusercontent.com/117917498/234520394-a0dcd014-4b82-4dac-85f9-ee11cc97d1da.gif) -->

<img src = "https://user-images.githubusercontent.com/117917498/234520394-a0dcd014-4b82-4dac-85f9-ee11cc97d1da.gif" width="30%" height="50%">

## Test Env.
The code is tested successfully at
- Python 3.8.10
- Linux 22.04 LTS
- YOLOv8
    - model: yolov8x.pt
### Hardware Requirements
<img src = "https://user-images.githubusercontent.com/117917498/234515405-5d4e3bbd-13a8-4bde-bb34-5271f0711340.png" width="50%" height="50%">
  
- OpenMANIPULATOR-X (RM-X52-TNM)
- U2D2 motor control board
- RealSense D435
- 3d printed D435 holder
 



## Requirements

### YOLOv8 Setting
- Install YOLOv8 on a machine by [ultralytics](https://github.com/ultralytics/ultralytics) git link

### Dynamixel Setting
- Install pip install dynamixel-sdk
 package by [ROBOTIS](https://github.com/ROBOTIS-GIT/DynamixelSDK)

```
pip install dynamixel-sdk
```

### Python Setting
- To run the python code, following pakages are necessary  
- to solve forward/inverse kinematics, we used [ikpy](https://github.com/Phylliade/ikpy)

```
pip install ikpy
pip install pyrealsense2	
pip install opencv-python
```



### Model data
- transfer learned 250 ~ 600 pics on each categroy based   
[aihub - 생활 폐기물 이미지](https://www.aihub.or.kr/aihubdata/data/view.do?currMenu=115&topMenu=100&aihubDataSe=realm&dataSetSn=140)


## How to Run
| Before run, set all the motor velocity and acceleration by [DYNAMIXEL Wizard ](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)to 70  
- for `gipper(motor ID = 15)`, use default speed and accel

<img src = "https://user-images.githubusercontent.com/117917498/234516918-aadd1a5c-15ec-47c0-8066-082eb4e9137c.png" width="50%" height="50%">

  
`main.py` : main code to run  
`teleop.py` : teleoperation arm

## Results

<img src = "https://user-images.githubusercontent.com/117917498/234520394-a0dcd014-4b82-4dac-85f9-ee11cc97d1da.gif" width="30%" height="50%">
