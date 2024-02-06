import gym
import os
import time
import LaRoboLiga24
import cv2
import pybullet as p
import cv2 as cv
import numpy as np
import math

CAR_LOCATION = [0,0,1.5]

BALLS_LOCATION = dict({
    'red': [7, 4, 1.5],
    'blue': [2, -6, 1.5],
    'yellow': [-6, -3, 1.5],
    'maroon': [-5, 9, 1.5]
})
BALLS_LOCATION_BONOUS = dict({
    'red': [9, 10, 1.5],
    'blue': [10, -8, 1.5],
    'yellow': [-10, 10, 1.5],
    'maroon': [-10, -9, 1.5]
})

HUMANOIDS_LOCATION = dict({
    'red': [11, 1.5, 1],
    'blue': [-11, -1.5, 1],
    'yellow': [-1.5, 11, 1],
    'maroon': [-1.5, -11, 1]
})

VISUAL_CAM_SETTINGS = dict({
    'cam_dist'       : 13,
    'cam_yaw'        : 0,
    'cam_pitch'      : -110,
    'cam_target_pos' : [0,4,0]
})


#os.chdir(os.path.dirname(os.getcwd()))
env = gym.make('LaRoboLiga24',
    arena = "arena2",
    car_location=CAR_LOCATION,
    ball_location=BALLS_LOCATION,  # toggle this to BALLS_LOCATION_BONOUS to load bonous arena
    humanoid_location=HUMANOIDS_LOCATION,
    visual_cam_settings=VISUAL_CAM_SETTINGS
)

"""
CODE AFTER THIS
"""

glob_velocity = 2

def spin():
    velocity = np.array([[-glob_velocity-1, glob_velocity + 1],
                         [-glob_velocity+1, glob_velocity-1]])
    return velocity

def spin_back():
    velocity = np.array([[glob_velocity+1, -glob_velocity - 1],
                         [glob_velocity-1, -glob_velocity+1]])
    return velocity


# The next 3 functions (contour distance, merge contour and agglomerative cluster) basically combine nearby contours as one contour
# This helps us get the entire goalpost as one contour and also the ball as one contour

def calculate_contour_distance(contour1, contour2): 
    x1, y1, w1, h1 = cv2.boundingRect(contour1)
    c_x1 = x1 + w1/2
    c_y1 = y1 + h1/2

    x2, y2, w2, h2 = cv2.boundingRect(contour2)
    c_x2 = x2 + w2/2
    c_y2 = y2 + h2/2

    return max(abs(c_x1 - c_x2) - (w1 + w2)/2, abs(c_y1 - c_y2) - (h1 + h2)/2)

def merge_contours(contour1, contour2):
    return np.concatenate((contour1, contour2), axis=0)

def agglomerative_cluster(contours, threshold_distance=40.0):
    current_contours = contours
    while len(current_contours) > 1:
        min_distance = None
        min_coordinate = None

        for x in range(len(current_contours)-1):
            for y in range(x+1, len(current_contours)):
                distance = calculate_contour_distance(current_contours[x], current_contours[y])
                if min_distance is None:
                    min_distance = distance
                    min_coordinate = (x, y)
                elif distance < min_distance:
                    min_distance = distance
                    min_coordinate = (x, y)

        if min_distance < threshold_distance:
            index1, index2 = min_coordinate
            current_contours[index1] = merge_contours(current_contours[index1], current_contours[index2])
            del current_contours[index2]
        else: 
            break

    return current_contours

def get_contours(img):
    mask = cv2.inRange(img, colors[color_list[color_index]][0], colors[color_list[color_index]][1])
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours = agglomerative_cluster([np.array(i) for i in contours])
    contours = sorted(contours, key=len)
    return contours

quit = ord('q')

env.open_grip()

colors = {"red": [np.array([0, 0.9, 0.4]), np.array([25, 1, 1], dtype=np.float64)],
              "yellow": [np.array([53, 0.44, 0.4]), np.array([65, 1, 1], dtype=np.float64)],
              'blue': [np.array([205, 0.8, 0.4]), np.array([245, 1, 1], dtype=np.float64)],
              "violet": [np.array([279, 0.8, 0.3]), np.array([330, 1, 1], dtype=np.float64)]
              }
color_list = list(colors.keys())

State=1
color_index = 0

while True:
    img = env.get_image(cam_height=0, dims=[512,512])
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    height, width, _ = img.shape

    cx = int(width / 2)
    cy = int(height / 2)
    hsv_correction = np.array([1/2, 255, 255])

    hsv_img = hsv_img / hsv_correction
    contours = get_contours(hsv_img)

    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)

        # Roundness is the measure of how close a shape is to a circle, as roundness becomes 1, the shape is closer to a circle
        try:
            roundness = 4 * np.pi * area / (perimeter * perimeter)
        except:
            roundness = 0
        centroid = np.round((np.sum(contour, axis=0) / len(contour))[0]).astype(int)
        if roundness > 0.7:
            rect = cv.minAreaRect(contour)
            rect_width = rect[1][0]
            if cx-(4000/rect_width) < centroid[0] and centroid[0] < cx+(4000/rect_width) and State == 1:
                env.move(vels=np.array([[0, 0],
                                        [0, 0]]))
                start_time= time.time()
                env.move(vels=np.array([[7, 7],
                                        [7, 7]]))
                prev_centroid = centroid[0]
                prev_area = area
                while area < 28000:
                    new_img = env.get_image(cam_height=0, dims=[512,512])
                    new_hsv_img = cv2.cvtColor(new_img, cv2.COLOR_BGR2HSV)
                    new_contours = get_contours(new_hsv_img / hsv_correction)
                    area = cv2.contourArea(new_contours[0])
                    new_centroid = np.round((np.sum(new_contours[0], axis=0) / len(new_contours[0]))[0]).astype(int)
                    if prev_area > area:
                        new_centroid[0] = prev_centroid
                    if new_centroid[0] < cx - 10:
                        env.move(vels = np.array([[7, 7*math.exp((cx - new_centroid[0])/512)],
                                         [7, 7*math.exp((cx - new_centroid[0])/512)]]))
                    elif new_centroid[0] > cx + 10:
                        env.move(vels = np.array([[7*math.exp((new_centroid[0] - cx)/512), 7],
                                         [7*math.exp((new_centroid[0] - cx)/512), 7]]))
                        
                    prev_centroid = new_centroid[0]
                    prev_area = area


                env.move(vels=np.array([[0, 0],
                    [0, 0]]))
                print('end')
                
                env.close_grip()
                end_time = time.time()
                env.move(vels=np.array([[-7, -7],
                                            [-7, -7]]))
                time.sleep(0.95*(end_time - start_time))
                env.move(vels=np.array([[0, 0],
                                        [0, 0]]))
                State = 2

        else:
            if cx-30 < centroid[0] < cx+30 and State == 2:
                env.open_grip()
                env.shoot()
                if color_index < 3:
                    color_index += 1

                State = 1


    keys = p.getKeyboardEvents()
    if State == 1:
        velocity = spin_back()
    else:
        velocity = spin_back()
    env.move(vels=velocity)

    if quit in keys:
        break

env.close()