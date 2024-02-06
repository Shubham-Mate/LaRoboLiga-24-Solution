import gym
import os
import time as t
import LaRoboLiga24
import cv2 as cv
import pybullet as p
import numpy as np
import math


CAR_LOCATION = [-25.5,0,1.5]



VISUAL_CAM_SETTINGS = dict({
    'cam_dist'       : 43,
    'cam_yaw'        : 0,
    'cam_pitch'      : -110,
    'cam_target_pos' : [0,4,0]
})


#os.chdir(os.path.dirname(os.getcwd()))
env = gym.make('LaRoboLiga24',
    arena = "arena1",
    car_location=CAR_LOCATION,
    visual_cam_settings=VISUAL_CAM_SETTINGS
)
# env.move(vels=[[6,6],
#                [6,6]])
# t.sleep(4)
# env.move(vels=[[0,0],
#                [0,0]])

# def wait(time=1):
#     t.sleep(time)




# def stop(time=1):
#     wait(time)
#     env.move(vels=[[0, 0], [0, 0]])






def move(mode='f', speed=5):
    if mode.lower() == "f":
        mat = [[speed, speed], [speed, speed]]
    elif mode.lower() == "r":
        mat = [[speed, -speed], [speed, -speed]]
    elif mode.lower()=="l":
        mat=[[-speed,speed],[-speed,speed]]
        
    else:
        pass
    env.move(vels=mat)


# def MoveHold(cnt):
#     x, y, w, h = cv.boundingRect(cnt)
#     x = x + w / 2 #mid pt
#     if x > 330:
#         move('r', (330 - x) / 30)
#     elif x < 270:
#         move('r', (270 - x) / 30)
#     else:
#         move('f', 10)
#         area = cv.contourArea(cnt)
#         if area > 36000:
#             global Holding
#             stop(.2)
            
            
            # Holding = True



while True:
    img = env.get_image(cam_height=0, dims=[550,550])
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret,binary_thresh = cv.threshold(gray, 175, 255, cv.THRESH_BINARY)
    # cv.imshow('Lanes', img)
    contours, _ = cv.findContours(binary_thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    color = cv.cvtColor(binary_thresh, cv.COLOR_GRAY2BGR)
    cv.drawContours(color, contours, -1, (0, 255, 0), 2)
    # cv.imshow('Lanes Detection', color)
    left_crop = binary_thresh[100:300 , 0:300]
    right_crop = binary_thresh[100:300 , 300:600]
    left_lane = np.array(left_crop)
    right_lane = np.array(right_crop)
    cv.imshow('left lane' , left_crop)
    cv.imshow('right lane' , right_crop)



    if ((math.fabs((cv.countNonZero(left_lane)) - (cv.countNonZero(right_lane)))) <= 2000 ):
        move('f',16)
    elif (((cv.countNonZero(left_lane)) - (cv.countNonZero(right_lane))) >= 2000):
        move('r', 16)
    elif (((cv.countNonZero(right_lane)) - (cv.countNonZero(left_lane))) >= 2000):
        move('r', -17)
    else:
        env.move(vels=[[0, 0], [0, 0]])
    k = cv.waitKey(1)
    if k == ord('q'):
        break
    
t.sleep(100)
env.close()