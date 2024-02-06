import gym
import os
import LaRoboLiga24
import cv2 as cv
import pybullet as p
import numpy as np
import math

CAR_LOCATION = [-25.5,0,1.5]


VISUAL_CAM_SETTINGS = dict({
    'cam_dist'       : 13,
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

"""
CODE AFTER THIS
"""

forward_vel = np.array([[6, 6],
                        [6, 6]])

def setVelocity(left_d, right_d, left_d_v, left_d_h, right_d_v, right_d_h):
    error1 = 0
    error2 = 0
    if (255 in right_d and 255 in left_d) or (not(255 in right_d) and not(255 in left_d)):
        return forward_vel
    elif 255 in right_d:
        for i, j in zip(right_d_h, range(4, 0, -1)):
            if 255 in i:
                error1 = j
                break
        for i, j in zip(right_d_v, range(1, 5)):
            print(i)
            if 255 in i:
                error2 = j
        print(1+min(error1,error2))
        return np.array([math.exp(-min(error1, error2)/3.5), math.log(math.e-1+min(error1,error2))])*forward_vel
    elif 255 in left_d:
        for i, j in zip(left_d_h, range(1, 5)):
            if 255 in i:
                error1 = j
        for i, j in zip(left_d_v, range(1, 5)):
            print(i)
            if 255 in i:
                error2 = j
        print(1+min(error1,error2))
        return np.array([math.log(math.e-1+min(error1,error2)), math.exp(-min(error1, error2)/3.5)])*forward_vel
        
quit = ord("q")

while True:
    img = env.get_image(cam_height=0, dims=[512,512])
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret,thresh1 = cv.threshold(img,200,255,cv.THRESH_BINARY)


    left_detector = thresh1[64:192, 128:192]
    right_detector = thresh1[64:128, 320:384]
    left_detectors_verticals = [thresh1[64 + 32*(i):64 + 32*(i+1), 128:192] for i in range(0, 4)]
    left_detectors_horizontal = [thresh1[64:128, 128 + 16*i : 128 + 16*(i+1)] for i in range(0, 4)]
    right_detector_verticals = [thresh1[64 + 32*i:64 + 32*(i+1), 320:384] for i in range(0, 4)]
    right_detectors_horizontal = [thresh1[64:128, 320 + 16*i : 320 + 16*(i+1)] for i in range(0, 4)]
    
    fin_image = cv.rectangle(thresh1, (128, 64), (192, 192), 127, 2)
    fin_image = cv.rectangle(fin_image, (384, 64), (320, 192), 127, 2)

    
    
    cv.imshow("img", fin_image)
    cv.waitKey(1)
    keys = p.getKeyboardEvents()
    env.move(vels=setVelocity(left_detector, right_detector, left_detectors_verticals, left_detectors_horizontal, right_detector_verticals, right_detectors_horizontal))
        
    if quit in keys:
        break

env.close()
