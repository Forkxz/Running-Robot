import cv2
import numpy as np
import time
import math
from Landmine import getAreaMaxContour1
from Color_define import color_dist
# from Robot_control import action_append,action_list
from CMDcontrol import action_append,action_list
# 将所有面积大于1的轮廓点拼接到一起    
def getSumContour(contours, area=1):
    contours_sum = None
    # print(len(contours))
    for c in contours:  #   初始化contours
        area_temp = math.fabs(cv2.contourArea(c))
        if (area_temp > area):
            contours_sum = c
            break
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        if (area_temp > area):
            contours_sum = np.concatenate((contours_sum, c), axis=0)  # 将所有面积大于1的轮廓点拼接到一起          
    return contours_sum

def door(Head):
    event_step_door = 0
    while True:
        door_left = 120; door_right = 520
        blobs = Head.imgs[int(0):int(200), int(door_left):int(door_right)].copy()   

        cv2.imshow("Head", blobs)
        cv2.waitKey(1)

        hsv = cv2.cvtColor(blobs, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (3, 3), 0)
        Imask = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
        Imask = cv2.erode(Imask, None, iterations=2)
        Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
        _, cnts, hierarchy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
        if cnts == None:
            action_append("Back1Run")
        else:
            cnt_sum = getSumContour(cnts, area=300)

        if event_step_door == 1:
            if cnt_sum is None:
                for i in range (10):
                    action_append("turn001L")
                    while len(action_list) != 0:
                        time.sleep(0.1)
                break
        else:
            if cnt_sum is not None:
                rect = cv2.minAreaRect(cnt_sum)
                box = np.int0(cv2.boxPoints(rect))

                box_width = abs(box[0,0] - box[2,0])/2

                center_x = (box[0,0]+box[2,0])/2 + door_left
                print("center_x ", center_x)

                cv2.drawContours(blobs, [box], -1, (0, 0, 255), 5)
                cv2.imshow('door sum', blobs)
                cv2.waitKey(1)
                print("boxwidth", box_width)
                x_dis = center_x - 320
                if box_width > 50:
                    if x_dis > 50:
                        action_append("Right3move")
                        action_append("HeadTurnMM")
                        while len(action_list) != 0:
                            time.sleep(0.1)
                    elif x_dis > 30:
                        action_append("Right02move")
                        action_append("HeadTurnMM")
                        while len(action_list) != 0:
                            time.sleep(0.1)
                    elif x_dis > -50 and x_dis < -30:
                        action_append("Left02move")
                        action_append("HeadTurnMM")
                        while len(action_list) != 0:
                            time.sleep(0.1)
                    elif x_dis < -50:
                        action_append("Left3move")
                        action_append("HeadTurnMM")
                        while len(action_list) != 0:
                            time.sleep(0.1)
                    else:
                        action_append("Forwalk02")  #Finish
                        action_append("HeadTurnMM")
                        while len(action_list) != 0:
                            time.sleep(0.1)
                        continue
                if box_width < 30:
                    action_append("Back1Run")
                    action_append("HeadTurnMM")
                    while len(action_list) != 0:
                        time.sleep(0.1)
            else:
                action_append("Forwalk01")
                action_append("Forwalk01")
                action_append("Forwalk01")
                action_append("Forwalk01")
                action_append("Forwalk01")
                action_append("HeadTurnMM")
                while len(action_list) != 0:
                    time.sleep(0.1)
                event_step_door = 1

if __name__ == '__main__':
    
    Chest_path = '../track_picture/test/1005chest5.png'
    Chest_img = cv2.imread(Chest_path,1)
    Chest_img = cv2.resize(Chest_img,(640,480))
    door(Chest_img)