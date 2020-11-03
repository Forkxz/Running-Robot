import cv2
import numpy as np
from Color_define import *
import math
from Robot_control import action_append


max_distance = 200
landmine_right_movecont = 0
landmine_left_movecont = 0
step_number = 0

class ShapeDetector:
    def __init__(self):
        pass
    def detect(self, c):
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
                # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"
        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        elif len(approx) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
        # if the shape is a pentagon, it will have 5 vertices
        elif len(approx) == 5:
            shape = "pentagon"
        # otherwise, we assume the shape is a circle
        else:
            shape = "circle"
        # return the name of the shape
        return shape

# 得到最大轮廓和对应的最大面积 
def getAreaMaxContour1(contours):    # 返回轮廓 和 轮廓面积
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 25:  #只有在面积大于25时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
    return area_max_contour, contour_area_max  # 返回最大的轮廓

def across_landmine_track_1(distance, step_num):
    global landmine_left_movecont
    global landmine_right_movecont
    
    #if abs(landmine_left_movecont-landmine_right_movecont) >= 4:
    #   print("test")
    #   #landmine_devition_monitor()
    if distance < max_distance and distance > 0:
        landmine_left_movecont +=1
        print("Left3move")
        #action_append("Left3move") #向左迈一步
    elif distance > -max_distance and distance < 0:
        landmine_right_movecont +=1
        print("Right3move")
        #action_append("Right3move") #向右迈一步
    elif distance == 0:
        landmine_left_movecont +=1
        print("Left3move")
        #action_append("Left3move") #向左或者向右迈一步都可以
    else:
        print("Forwalk02")
        #action_append("Forwalk02") #向前进
        step_num += 1
    return step_num

def across_landmine_track_2(distance, step_num):
    global landmine_left_movecont
    global landmine_right_movecont

    if abs(distance) < max_distance:
        print("Forwalk02")
        #action_append("Forwalk02") #向前进
        step_num += 1
    elif distance < -max_distance :
        landmine_left_movecont +=1
        print("Left3move")
        #action_append("Left3move") #向左迈一步
    elif distance < max_distance:
        landmine_right_movecont +=1
        print("Right3move")
        #action_append("Right3move") #向右迈一步都可以
    return step_num

def landmine(ChestOrg_img):
    # global DIM, ChestOrg_img, HeadOrg_img, event_state, step_number

    org_img_copy = np.rot90(ChestOrg_img).copy()
    roi = org_img_copy[450:600,0:480]

    # cv2.imshow("landmine roi", roi)
    # cv2.waitKey(1)

    roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    roi_mask = cv2.inRange(roi_hsv, color_dist['landmine']['Lower'], color_dist['landmine']['Upper'])

    _, contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    min_size = 1000
    max_size = 4500
    selec_cnts = []
    landmine_center = []

    sd = ShapeDetector()
    #筛选轮廓
    if contours is not None:
        for c in contours:
            shape = sd.detect(c)
            #print(shape)
            #轮廓可视化
            M = cv2.moments(c)
            cX = int(M["m10"] / (M["m00"] + 0.001))
            cY = int(M["m01"] / (M["m00"] + 0.001))
            c = c.astype("int")
            #cv2.drawContours(roi, [contours[i]], -1, (0, 255, 0), -1)
            #cv2.putText(roi, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            #cv2.imshow("shapename",roi)
            area = cv2.contourArea(c) #计算轮廓面积
            print(area)
                #rect = cv2.minAreaRect(contours[i])#最小外接矩形
                #area = rect[1][0] * rect[1][1]
                #print(area)
                #box = cv2.boxPoints(rect) # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
                #box = np.int0(box)
                #cv2.drawContours(roi, [box], 0, (0, 0, 255), 1) #画出来
                #cv2.imshow("hehe",roi)
                #筛选矩形且大小在范围内的轮廓
            if (shape == "rectangle" or "circle" or "pentagon") and (area > min_size) and (area < max_size):
                selec_cnts.append(c)
        
        for c in selec_cnts:
            rect = cv2.minAreaRect(c)#最小外接矩形
            box = cv2.boxPoints(rect)
            box = np.int0(box) # 画出来

            cv2.drawContours(roi, [box], 0, (255, 0, 0), 1)
            landmine_center.append((int(rect[0][0]),int(rect[0][1])))
            
        for i in landmine_center:
            cv2.circle(roi, i, 1, (0,0,255), 4)
        cv2.imshow("min_rectangle",roi)

        contour_num = len(selec_cnts)
        if contour_num == 0:
            distance = 500 #随便设的数值，只要够独一无二就行
            step_number = across_landmine_track_1(distance,step_number)
        elif contour_num == 1:
            distance = landmine_center[0][0] - 240
            step_number = across_landmine_track_1(distance,step_number)
        elif contour_num == 2:
            center_point_x_between_twolandmine = (landmine_center[0][0] + landmine_center[1][0] ) / 2
            distance = center_point_x_between_twolandmine - 240
            step_number = across_landmine_track_2(distance,step_number)
        else:
            first_large_y = 0
            first_large_x = 0
            seconde_large_y = 0
            seconde_large_x = 0
            for i in landmine_center:
                if i[1] > first_large_y:
                    first_large_y = i[1]
                    first_large_x = i[0]
                if i[1] > seconde_large_y and i[1]!= first_large_y:
                    seconde_large_y = i[1]
                    seconde_large_x = i[0]
            center_point_x_between_twolandmine = (first_large_x + seconde_large_x) / 2
            step_number = across_landmine_track_2(distance,step_number)
    
    else:
        print("Forwalk02")
        step_number += 1