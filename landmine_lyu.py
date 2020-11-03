import numpy as np
import cv2
import threading
import time
from CMDcontrol import action_append,action_list
# from Robot_control import action_append,action_list
import math


def detectrectangle(contour):
    shape = "unidentified"
    peri = cv2.arcLength(contour, True)
    #print(peri)
    approx = cv2.approxPolyDP(contour, 0.03 * peri, True)  #将曲线转化为多个线段相连接
    # 三角形会有三条线段
    if len(approx) == 3:
        shape = "triangle"
        # 正方形或者长方形有四条线段
    elif len(approx) == 4:
        # 当时四边形的时候通过宽高比来判断正方形和长方形
        (x, y, w, h) = cv2.boundingRect(approx)
        ar = w / float(h)
        if ar >= 0.95 and ar <= 1.05:
            shape = "square"
        else:
            shape = "rectangle"
        # 多边形有五条线段
    elif len(approx) == 5:
        shape = "pentagon"
    # 圆形有大于五条的线段
    else:
        shape = "circle"
    # 返回图形名称
    return shape


##去除contours数列中在delete_list中的轮廓
def delete_contours(contours, delete_list):
    delta = 0
    for i in range(len(delete_list)):
        del contours[delete_list[i] - delta]
        delta = delta + 1
    return contours


def landmine_devition_monitor(landmine_right_movecont,landmine_left_movecont):
    if landmine_left_movecont - landmine_right_movecont >= 12:
        print("Right02move")
        action_append("Right02move")  #向右迈一步
        action_append("Right02move")  #向右迈一步
        action_append("Right02move")  #向右迈一步
        landmine_right_movecont += 3
    elif landmine_right_movecont - landmine_left_movecont >= 12:
        print("Left02move")
        action_append("Left02move")  #向左迈一步
        action_append("Left02move")  #向左迈一步
        action_append("Left02move")  #向左迈一步
        landmine_left_movecont += 3
    return landmine_right_movecont,landmine_left_movecont


def across_landmine_track_1(distance, step_num, landmine_right_movecont, landmine_left_movecont):
    if distance < landmine_max_distance and distance > 0:
        if abs(landmine_left_movecont - landmine_right_movecont) >= 12:
            print("test")
            landmine_right_movecont,landmine_left_movecont = landmine_devition_monitor(landmine_right_movecont,landmine_left_movecont)
        else:
            landmine_left_movecont += 1
            print("Left02move")
            action_append("Left02move")  #向左迈一步
    elif distance > -landmine_max_distance and distance < 0:
        if abs(landmine_left_movecont - landmine_right_movecont) >= 12:
            print("test")
            landmine_right_movecont,landmine_left_movecont = landmine_devition_monitor(landmine_right_movecont,landmine_left_movecont)
        else:
            landmine_right_movecont += 1
            print("Right02move")
            action_append("Right02move")  #向右迈一步
    elif distance == 0:
        if abs(landmine_left_movecont - landmine_right_movecont) >= 12:
            print("test")
            landmine_right_movecont,landmine_left_movecont = landmine_devition_monitor(landmine_right_movecont,landmine_left_movecont)
        else:
            landmine_left_movecont += 1
            print("Left02move")
            action_append("Left02move")  #向左迈一步都可以
    else:
        #if step_number % 6 == 0 and step_number != 0:
        #    print("turn000R")
        #    action_append("turn000R")
        print("Forwalk02")
        action_append("Forwalk02")  #向前进
        action_append("turn000R")
        step_num += 1
    return step_num,landmine_right_movecont,landmine_left_movecont


def across_landmine_track_2(distance, step_num,landmine_right_movecont,landmine_left_movecont):
    """return step_num,landmine_right_movecont,landmine_left_movecont
    """
    if abs(distance) <= landmine_max_distance:
        #if step_number % 6 == 0 and step_number != 0 :
        #    print("turn000R")
        #    action_append("turn000R")
        print("Forwalk02")
        action_append("Forwalk02")  #向前进
        action_append("turn000R")
        step_num += 1
    elif distance < -landmine_max_distance:
        if abs(landmine_left_movecont - landmine_right_movecont) >= 12:
            print("test")
            landmine_right_movecont,landmine_left_movecont = landmine_devition_monitor(landmine_right_movecont,landmine_left_movecont)
        else:
            landmine_left_movecont += 1
            print("Left02move")
            action_append("Left02move")  #向左迈一步
    elif distance > landmine_max_distance:
        if abs(landmine_left_movecont - landmine_right_movecont) >= 12:
            print("test")
            landmine_right_movecont,landmine_left_movecont = landmine_devition_monitor(landmine_right_movecont,landmine_left_movecont)
        else:
            landmine_right_movecont += 1
            print("Right02move")
            action_append("Right02move")  #向右迈一步

    return step_num,landmine_right_movecont,landmine_left_movecont


#######################################################################


def landmine_lyu(Chest):
    global landmine_end_flag_judge
    global landmine_max_distance
    landmine_max_distance = 150  #############

    landmine_start_flag = False
    landmine_end_flag_judge = False
    landmine_right_movecont = 0
    landmine_left_movecont = 0
    landmine_end_flag = 0
    step_number = 0

    landmine_judge_loop_cont = 0
    landmine_pass_loop_cont = 0

    # lanemine_correcion(Head)
    #开始地雷关卡开始判断
    while (True):
        # time.sleep(1)
        if len(action_list)==0:
            landmine_judge_loop_cont += 1
            origin_img = np.rot90(Chest.imgs).copy()
            # cv2.imshow("origin", origin_img)
            # cv2.waitKey(1)
            img_test = origin_img.copy()
            roi = img_test[410:600, 0:480]  #取感兴趣区域
            cv2.imshow("roi", roi)
            cv2.waitKey(1)
            roi_shape = roi.shape

            #颜色阈值

            color_high = np.array([95,120,55])
            color_low = np.array([65,44,20])
            # color_high = 50
            # color_low = 0

            #转到灰度空间
            # roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            roi_hsv = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
            # cv2.imshow("roi_hsv",roi_hsv)

            #颜色筛选
            # roi_mask = cv2.inRange(roi_gray, color_low, color_high)
            roi_mask = cv2.inRange(roi_hsv, color_low, color_high)
            cv2.imshow("roi_mask_inrange",roi_mask)

            ##腐蚀
            #roi_mask = cv2.erode(roi_mask, np.ones((3,3)), iterations=1)
            #cv2.imshow("roi_mask_erode",roi_mask)
            #膨胀
            roi_mask = cv2.dilate(roi_mask, np.ones((3, 3)), iterations=1)
            # cv2.imshow("roi_mask",roi_mask)

            #找轮廓
            _, contours,_= cv2.findContours(roi_mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_NONE)
            #image = cv2.drawContours(roi, contours, -1,(255,0,0),1 )
            # cv2.imshow("contours",image)

            #设置最大最小面积
            min_size = 500
            max_size = 4500
            delete_list = []
            #print(len(contours))
            #筛选轮廓
            for i in range(len(contours)):
                shape = detectrectangle(contours[i])  #检测轮廓形状
                #print(shape)
                #轮廓可视化
                # M = cv2.moments(contours[i])
                # cX = int(M["m10"] / (M["m00"] + 0.001))
                # cY = int(M["m01"] / (M["m00"] + 0.001))
                contours[i] = contours[i].astype("int")
                # cv2.drawContours(roi, [contours[i]], -1, (0, 255, 0), -1)
                # cv2.putText(roi, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # cv2.imshow("shapename",roi)
                area = cv2.contourArea(contours[i])  #计算轮廓面积
                #print(area)
                #rect = cv2.minAreaRect(contours[i])#最小外接矩形
                #area = rect[1][0] * rect[1][1]
                #print(area)
                #box = cv2.boxPoints(rect) # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
                #box = np.int0(box)
                #cv2.drawContours(roi, [box], 0, (0, 0, 255), 1) #画出来
                #cv2.imshow("hehe",roi)
                #筛选矩形且大小在范围内的轮廓
                if shape == "rectangle" or shape == "pentagon" or shape == "circle":
                    shape_flag = True
                else:
                    shape_flag = False
                if shape_flag != True or (area < min_size) or (area > max_size):
                    #print(i)
                    delete_list.append(i)

            contours = delete_contours(contours, delete_list)
            #print(len(contours)) #看看识别成功的轮廓数量

            #判断符合情况的轮廓数量
            if len(contours) >= 1:
                landmine_start_flag = True
            else:
                landmine_start_flag = False
            #for i in contours:
            #print(cv2.arcLength(i,True))

            #展示识别成功的轮廓
            # image = cv2.drawContours(roi, contours, -1,(0,0,255),1 )
            # cv2.imshow("contours5",image)

            print(landmine_start_flag)

            if landmine_start_flag == True:
                print("start passing process")
                break
            else:
                print("Forwalk02")
                action_append("Forwalk02")

            if landmine_judge_loop_cont == 4:  ### 这个地方可能要改，取决于上个关卡结束至地雷关卡开始的距离（看看差了几个Forwalk02）
                print("landmine judge failed, start passing process")
                landmine_start_flag = True
                break

    #开始进行过地雷关卡
    while (landmine_start_flag):
        # time.sleep(2)
        if len(action_list)==0:
            landmine_pass_loop_cont += 1
            # if step_number % 2 == 0 and step_number != 0:
            #    print("turn000R")
            #    action_append("turn000R")
            landmine_center = []
            origin_img = Chest.imgs.copy()
            origin_img = np.rot90(origin_img)
            # cv2.imshow("origin", origin_img)
            img_test = origin_img.copy()
            roi = img_test[410:600, 0:480]  #取感兴趣区域要近一些从脚底开始
            cv2.imshow("roi", roi)
            # cv2.waitKey(1)
            roi_shape = roi.shape

            ###颜色阈值 根据实际情况调整
            color_high = np.array([95,120,55])
            color_low = np.array([65,44,20])
            # color_high = 50
            # color_low = 0

            #转到hsv空间
            # roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            roi_hsv = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
            #cv2.imshow("roi_hsv",roi_hsv)

            #颜色筛选
            roi_mask = cv2.inRange(roi_hsv, color_low, color_high)
            cv2.imshow("roi_mask_inrange",roi_mask)

            ##腐蚀
            #roi_mask = cv2.erode(roi_mask, np.ones((3,3)), iterations=1)
            #cv2.imshow("roi_mask_erode",roi_mask)
            #膨胀
            roi_mask = cv2.dilate(roi_mask, np.ones((3, 3)), iterations=1)
            #cv2.imshow("roi_mask",roi_mask)

            #找轮廓
            _, contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_NONE)
            image = cv2.drawContours(roi, contours, -1,(255,0,0),1 )
            cv2.imshow("contours",image)
            cv2.waitKey(1)
            ###设置最大最小面积 最大面积和最小面积都要稍微大一些，根据实际情况调整
            min_size = 500
            max_size = 4500
            delete_list = []
            shape_flag = False
            #筛选轮廓
            if len(contours) != 0:
                for i in range(len(contours)):
                    shape = detectrectangle(contours[i])  #检测轮廓形状
                    #print(shape)
                    #轮廓可视化
                    # M = cv2.moments(contours[i])
                    # cX = int(M["m10"] / (M["m00"] + 0.001))
                    # cY = int(M["m01"] / (M["m00"] + 0.001))
                    contours[i] = contours[i].astype("int")
                    # cv2.drawContours(roi, [contours[i]], -1, (0, 255, 0), -1)
                    # cv2.putText(roi, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    #cv2.imshow("shapename",roi)
                    area = cv2.contourArea(contours[i])  #计算轮廓面积
                    #print(area)
                    #rect = cv2.minAreaRect(contours[i])#最小外接矩形
                    #area = rect[1][0] * rect[1][1]
                    #print(area)
                    #box = cv2.boxPoints(rect) # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
                    #box = np.int0(box)
                    #cv2.drawContours(roi, [box], 0, (0, 0, 255), 1) #画出来
                    #cv2.imshow("hehe",roi)
                    #筛选矩形且大小在范围内的轮廓
                    if shape == "rectangle" or shape == "circle" or shape == "pentagon":
                        shape_flag = True
                    else:
                        shape_flag = False
                    if shape_flag != True or (area < min_size) or (area >
                                                                max_size):
                        delete_list.append(i)

                contours = delete_contours(contours, delete_list)
                contour_num = len(contours)  #筛选后的轮廓数量
                print("the contour_num is" + str(contour_num))
                #print(len(contours)) #看看识别成功的轮廓数量
                for i in contours:
                    rect = cv2.minAreaRect(i)  #最小外接矩形
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)  # 画出来

                    #cv2.drawContours(roi, [box], 0, (255, 0, 0), 1)
                    landmine_center.append((int(rect[0][0]), int(rect[0][1])))

                print("landmine center is " + str(landmine_center))
                #画点
                # for i in landmine_center:
                # cv2.circle(roi, i, 1, (0,0,255), 4)
                #cv2.imshow("min_rectangle",roi)

                if contour_num == 0:
                    distance = 500  #随便设的数值，只要够独一无二就行
                    if landmine_end_flag == 2:
                        print("begin end")
                        break
                    elif step_number > 8:
                        landmine_end_flag += 1
                        print(landmine_end_flag)
                        step_number,landmine_right_movecont,landmine_left_movecont = across_landmine_track_1(
                            distance, step_number,landmine_right_movecont,landmine_left_movecont)
                    else:
                        step_number,landmine_right_movecont,landmine_left_movecont = across_landmine_track_1(
                            distance, step_number,landmine_right_movecont,landmine_left_movecont)
                elif contour_num == 1:
                    landmine_end_flag = 0
                    distance = landmine_center[0][0] - 240
                    step_number,landmine_right_movecont,landmine_left_movecont = across_landmine_track_1(distance, step_number,landmine_right_movecont,landmine_left_movecont)
                elif contour_num == 2:
                    landmine_end_flag = 0
                    center_point_x_between_twolandmine = (
                        landmine_center[0][0] + landmine_center[1][0]) / 2
                    distance = center_point_x_between_twolandmine - 240
                    step_number,landmine_right_movecont,landmine_right_movecont = across_landmine_track_2(distance, step_number,landmine_right_movecont,landmine_left_movecont)
                else:
                    landmine_end_flag = 0
                    first_large_y = 0
                    first_large_x = 0
                    seconde_large_y = 0
                    seconde_large_x = 0
                    for i in landmine_center:
                        if i[1] > first_large_y:
                            first_large_y = i[1]
                            first_large_x = i[0]
                        if i[1] > seconde_large_y and i[0] != first_large_x:
                            seconde_large_y = i[1]
                            seconde_large_x = i[0]

                    center_point_x_between_twolandmine = (first_large_x +
                                                        seconde_large_x) / 2
                    distance = center_point_x_between_twolandmine
                    step_number,landmine_right_movecont,landmine_right_movecont = across_landmine_track_2(distance, step_number,landmine_right_movecont,landmine_right_movecont)
            else:
                if landmine_end_flag == 2:
                    print("begin end")
                    break
                elif step_number > 8:
                    landmine_end_flag += 1
                    print(landmine_end_flag)
                    print("Forwalk02")
                    action_append("Forwalk02")
                    action_append("turn000R")
                    step_number += 1
                else:
                    print("Forwalk02")
                    action_append("Forwalk02")
                    action_append("turn000R")
                    step_number += 1
            #停止关卡判断
            if landmine_end_flag == 2 and step_number == 15:
                movement_sum = landmine_left_movecont - landmine_right_movecont
                if movement_sum > 0:
                    landmine_right_movecont += 1
                    print("Right02move")
                    action_append("Right02move")
                elif movement_sum < 0:
                    landmine_left_movecont += 1
                    print("Left02move")
                    action_append("Left02move")
                elif movement_sum == 0:
                    print("Robot Debug: Pass!")
                    landmine_end_flag_judge = True
                    break

            # if step_number == 21:
            #     print("landmine pass end judge failed, end landmine misson")
            #     landmine_end_flag_judge = True
            #     break
