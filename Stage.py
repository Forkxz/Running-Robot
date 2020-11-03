import cv2
import numpy as np
import time
import threading
import math
import datetime
from CMDcontrol import action_append
# from Robot_control import action_append
color_range = {
    'yellow_door': [(10, 43, 46), (34, 255, 255)],
    'red_floor': [(0, 130, 150), (10, 230, 255)],
    'green_floor': [(60, 60, 60), (90, 120, 140)],
    'blue_floor': [(100, 70, 60), (140, 180, 160)],
    'red_floor2': [(156, 43, 46), (180, 255, 255)],
    'green_bridge': [(35, 43, 20), (100, 255, 255)],
    'yellow_hole': [(10, 70, 46), (34, 255, 255)],
    'black_hole': [(40, 80, 200), (70, 160, 255)],
    'black_gap': [(0, 0, 0), (180, 255, 100)],
    'black_dir': [(0, 0, 0), (180, 255, 46)],
    'blue': [(100, 120, 100), (140, 240, 200)],
    'black_door': [(0, 0, 0), (180, 255, 46)],
    'white_floor': [(30, 40, 100), (100, 100, 200)]
}


def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    #area_max_contour = np.array([[[0, 0]], [[0, 1]], [[1, 1]], [[1, 0]]])
    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 25:  #只有在面积大于25时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
    return area_max_contour, contour_area_max  # 返回最大的轮廓

def stagePosition(Head):
    x_central = 320

    Horg_img = Head.imgs.copy()
    #cropImg = Horg_img[240:480, 0:640, :]  #裁剪图像
    cropImg = Horg_img
    #cv2.imwrite('de.jpg',cropImg) #写入图像路径
    frame_gauss = cv2.GaussianBlur(cropImg, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
    frame_door = cv2.inRange(
        frame_hsv, color_range['red_floor'][0],
        color_range['red_floor'][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN,
                              np.ones((3, 3), np.uint8))  # 开运算 去噪点
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE,
                              np.ones((10, 10), np.uint8))  # 闭运算 封闭连接
    _, contours, hierarchy = cv2.findContours(closed, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_NONE)  # 找出轮廓
    areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
    cv2.drawContours(cropImg, areaMaxContour, -1, (0, 0, 255), 3)
    cv2.imshow("crop_Img", cropImg)
    # cv2.waitKey(1)
    if areaMaxContour is not None:
        rect = cv2.minAreaRect(areaMaxContour)  #最小外接矩形
        box = np.int0(cv2.boxPoints(rect))  #最小外接矩形的四个顶点
        cv2.drawContours(cropImg, [box], 0, (255, 0, 0), 5)
        x_central = (box[0, 0] + box[2, 0]) / 2
    else:
        x_central = 350
    print("x_central of stage position is :" + str(x_central))
    return x_central

def stagePosition_y(Head):
    y_central = 300
    Horg_img = Head.imgs.copy()
    #cropImg = Horg_img[240:480,0:640,:]  #裁剪图像
    cropImg = Horg_img
    #cv2.imwrite('de.jpg',cropImg) #写入图像路径
    frame_gauss = cv2.GaussianBlur(cropImg, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
    frame_door = cv2.inRange(frame_hsv, color_range['red_floor'][0],
                                        color_range['red_floor'][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))  # 闭运算 封闭连接
    _, contours, hierarchy = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
    areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
    cv2.drawContours(cropImg,areaMaxContour,-1,(0,0,255),3)
    if areaMaxContour is not None:
        rect = cv2.minAreaRect(areaMaxContour)#最小外接矩形
        box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
        cv2.drawContours(cropImg, [box], 0, (255,0,0), 5)
        y_central = (box[0,1] + box[2,1])/2
    else:
        y_central = 300
    print("y_central of stage position is :" + str(y_central))
    return y_central

def Stage(Chest, Head):
    stage_clear = 0
    num = 0
    red_num = 0
    action = 0
    while (True):
        while stage_clear == 0:
            time.sleep(1)
            org_img = Head.imgs.copy()
            # r_w = 640
            # r_h = 480
            # border = cv2.copyMakeBorder(org_img,12,12,16,16,borderType=cv2.BORDER_CONSTANT,value=(255, 255, 255))  # 扩展白边，防止边界无法识别
            #cv2.imwrite('border.jpg', border)
            cropImg = org_img
            #cv2.imwrite('de.jpg',cropImg) #写入图像路径
            frame_gauss = cv2.GaussianBlur(cropImg, (3, 3), 0)  # 高斯模糊
            frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
            #h, s, v = cv2.split(frame_hsv)  # 分离出各个HSV通道
            #v = cv2.equalizeHist(v)  # 直方图化
            #Frame = cv2.merge((h, s, v))  # 合并三个通道
            #对图片采样
            # low = [0, 0, 0]
            # for i in range(0, 400):
            #     for n in range(0, 55):
            #         low = low + frame_hsv[170 + n, 120 + i]
            # low = low/22000 #求采样结果的均值
            # low = low * 0.8 #根据均值算图片中目标区域的色彩阈值，low乘以的数需根据目标区域与周围区域颜色的差别大小调整
            # high = low * 2.4
            #frame_door1 = cv2.inRange(frame_hsv, low,
            #high)  # 对原图像和掩模(颜色的字典)进行位运算
            frame_door1 = cv2.inRange(frame_hsv, color_range['blue_floor'][0],
                                                color_range['blue_floor'][1])  # 对原图像和掩模(颜色的字典)进行位运算
            frame_door2 = cv2.inRange(frame_hsv, color_range['green_floor'][0],
                                                color_range['green_floor'][1])  # 对原图像和掩模(颜色的字典)进行位运算
            frame_door3 = cv2.inRange(frame_hsv, color_range['red_floor'][0],
                                                color_range['red_floor'][1])  # 对原图像和掩模(颜色的字典)进行位运算
            #frame_door = cv2.add(frame_door1, frame_door2)
            #cv2.imwrite('door.jpg', frame_door1)
            opened = cv2.morphologyEx(frame_door1, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))  # 闭运算 封闭连接
            #cv2.imwrite('closed.jpg', closed)
            # print(closed)
            opened2 = cv2.morphologyEx(frame_door2, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
            closed2 = cv2.morphologyEx(opened2, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))  # 闭运算 封闭连接

            opened3 = cv2.morphologyEx(frame_door3, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
            closed3 = cv2.morphologyEx(opened3, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))  # 闭运算 封闭连接

            _, contours, hierarchy = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
            cv2.drawContours(cropImg,areaMaxContour,-1,(0,0,255),3)
            if areaMaxContour is not None:
                rect = cv2.minAreaRect(areaMaxContour)#最小外接矩形
                box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
                cv2.drawContours(cropImg, [box], 0, (255,0,0), 5)
            else:
                print('too far')
                red_num = red_num + 1
                area_max = 0

            _, contours2, hierarchy2 = cv2.findContours(closed2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            areaMaxContour2, area_max2 = getAreaMaxContour(contours2)  # 找出最大轮廓
            cv2.drawContours(cropImg,areaMaxContour2,-1,(0,0,255),3)
            if areaMaxContour2 is not None:
                rect2 = cv2.minAreaRect(areaMaxContour2)#最小外接矩形
                box2 = np.int0(cv2.boxPoints(rect2))#最小外接矩形的四个顶点
                cv2.drawContours(cropImg, [box2], 0, (255,0,0), 5)
            else:
                action_append('Forwalk01')
                print('too far')
                area_max2 = 0
                action = 1

            _, contours3, hierarchy3 = cv2.findContours(closed3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            areaMaxContour3, area_max3 = getAreaMaxContour(contours3)  # 找出最大轮廓
            cv2.drawContours(cropImg,areaMaxContour3,-1,(0,0,255),3)
            if areaMaxContour3 is not None:
                rect3 = cv2.minAreaRect(areaMaxContour3)#最小外接矩形
                box3 = np.int0(cv2.boxPoints(rect3))#最小外接矩形的四个顶点
                cv2.drawContours(cropImg, [box3], 0, (255,0,0), 5)
            else:
                action_append('Forwalk01')
                print('too far')
                area_max3 = 0
                action = 1

            #print(areaMaxContour)
            cv2.imshow('contour', org_img)
            cv2.waitKey(1)
            percent = round(100 * (area_max + area_max2 + area_max3) / (640 * 480), 2)  # 最大轮廓的百分比
            # if box3 is not None:
            #     x_central = (box3[0,0] + box3[2,0])/2
            # else:
            #     print('no red floor')
            # print(percent)
            #print(box[0,0])
            if action == 1:
                x_central = stagePosition(Head)
            else:
                action_append('Forwalk01')
                x_central = (box3[0,0] + box3[2,0])/2
            print(x_central)
            forward = 0
            left = 0
            right = 0
            if x_central < 250:
                left = 1
            elif x_central >  270:
                right = 1
            else:
                forward = 1
            if right == 1:
                action_append('Right02move')
                action_append('Right02move')
            elif left == 1:
                action_append('Left02move')
                action_append('Left02move')
            else:
                action_append('Forwalk02')
            if red_num == 5:
                num = 1
            else:
                pass
            if percent > 7:
                close_center = 0
                center_num = 0
                while((close_center == 0) & (center_num < 6)):
                    x_central = stagePosition(Head)
                    if x_central < 250:
                        action_append('Left02move')
                        center_num = center_num + 1
                    elif x_central >  270:
                        action_append('Right02move')
                        center_num = center_num + 1
                    else:
                        close_center = 1
                close_forward = 0
                while(close_forward == 0):
                    action_append('Forwalk02')
                    x_central = stagePosition(Head)
                    if x_central < 250:
                        action_append('Left02move')
                    elif x_central >  270:
                        action_append('Right02move')
                    else:
                        pass
                    y_central = stagePosition_y(Head)
                    if y_central >= 330:
                        close_forward = 1
                        print('y_central:', y_central)
                        print('stand by ready')
                    else:
                        action_append('Forwalk02')
                action_append('Forwalk02')
                action_append('shanglouti_v1')
                time.sleep(1)
                stage_position = stagePosition(Head)
                if (stage_position > 340) & (stage_position < 380):
                    action_append('Right02move')
                elif (stage_position >= 380) & (stage_position < 420):
                    action_append('Right02move')
                    action_append('Right02move')
                elif stage_position >= 420:
                    action_append('Right02move')
                    action_append('Right02move')
                    action_append('Right02move')
                elif (stage_position < 300) & (stage_position > 260):
                    action_append('Left02move')
                elif (stage_position <= 260) & (stage_position > 220):
                    action_append('Left02move')
                    action_append('Left02move')
                elif stage_position <= 220:
                    action_append('Left02move')
                    action_append('Left02move')
                    action_append('Left02move')
                else:
                    pass
                action_append('Forwalk02')
                action_append('shanglouti_v1')
                time.sleep(1)
                stage_position = stagePosition(Head)
                if (stage_position > 340) & (stage_position < 380):
                    action_append('Right02move')
                elif (stage_position >= 380) & (stage_position < 420):
                    action_append('Right02move')
                    action_append('Right02move')
                elif stage_position >= 420:
                    action_append('Right02move')
                    action_append('Right02move')
                    action_append('Right02move')
                elif (stage_position < 300) & (stage_position > 260):
                    action_append('Left02move')
                elif (stage_position <= 260) & (stage_position > 220):
                    action_append('Left02move')
                    action_append('Left02move')
                elif stage_position <= 220:
                    action_append('Left02move')
                    action_append('Left02move')
                    action_append('Left02move')
                else:
                    pass
                action_append('Forwalk02')
                action_append('shanglouti_v1')
                time.sleep(1)
                action_append('Forwalk00')
                action_append('Forwalk00')
                action_append('xialouti')
                time.sleep(1)
                action_append('turn004L')
                action_append('xialouti')
                time.sleep(1)
                action_append('turn004L')
                action_append('Left02move')
                action_append('Left02move')
                action_append('turn004L')
                # stage_position = stagePosition_Chest(Chest)
                # if (stage_position < 220) & (stage_position > 190):
                #     action_append('Left02move')
                # elif stage_position <= 190:
                #     action_append('Left02move')
                #     action_append('Left02move')
                # elif (stage_position > 260) & (stage_position < 290):
                #     action_append('Right02move')
                # elif stage_position >= 290:
                #     action_append('Right02move')
                #     action_append('Right02move')
                # else:
                #     pass
                action_append('Forwalk02')
                action_append('Forwalk02')
                action_append('Forwalk02')
                action_append('Forwalk02')
                action_append('Forwalk02')
                stage_clear = 1
            elif num == 1:
                close_center = 0
                center_num = 0
                while((close_center == 0) & (center_num < 6)):
                    x_central = stagePosition(Head)
                    if x_central < 250:
                        action_append('Left02move')
                        center_num = center_num + 1
                    elif x_central >  270:
                        action_append('Right02move')
                        center_num = center_num + 1
                    else:
                        close_center = 1
                close_forward = 0
                while(close_forward == 0):
                    action_append('Forwalk02')
                    x_central = stagePosition(Head)
                    if x_central < 250:
                        action_append('Left02move')
                    elif x_central >  270:
                        action_append('Right02move')
                    else:
                        pass
                    y_central = stagePosition_y(Head)
                    if y_central >= 330:
                        close_forward = 1
                        print('y_central:', y_central)
                        print('stand by ready')
                    else:
                        action_append('Forwalk02')
                action_append('Forwalk02')
                action_append('shanglouti_v1')
                time.sleep(1)
                stage_position = stagePosition(Head)
                if (stage_position > 340) & (stage_position < 380):
                    action_append('Right02move')
                elif (stage_position >= 380) & (stage_position < 420):
                    action_append('Right02move')
                    action_append('Right02move')
                elif stage_position >= 420:
                    action_append('Right02move')
                    action_append('Right02move')
                    action_append('Right02move')
                elif (stage_position < 300) & (stage_position > 260):
                    action_append('Left02move')
                elif (stage_position <= 260) & (stage_position > 220):
                    action_append('Left02move')
                    action_append('Left02move')
                elif stage_position <= 220:
                    action_append('Left02move')
                    action_append('Left02move')
                    action_append('Left02move')
                else:
                    pass
                action_append('Forwalk02')
                action_append('shanglouti_v1')
                time.sleep(1)
                stage_position = stagePosition(Head)
                if (stage_position > 340) & (stage_position < 380):
                    action_append('Right02move')
                elif (stage_position >= 380) & (stage_position < 420):
                    action_append('Right02move')
                    action_append('Right02move')
                elif stage_position >= 420:
                    action_append('Right02move')
                    action_append('Right02move')
                    action_append('Right02move')
                elif (stage_position < 300) & (stage_position > 260):
                    action_append('Left02move')
                elif (stage_position <= 260) & (stage_position > 220):
                    action_append('Left02move')
                    action_append('Left02move')
                elif stage_position <= 220:
                    action_append('Left02move')
                    action_append('Left02move')
                    action_append('Left02move')
                else:
                    pass
                action_append('Forwalk02')
                action_append('shanglouti_v1')
                time.sleep(1)
                action_append('Forwalk00')
                action_append('Forwalk00')
                action_append('xialouti')
                time.sleep(1)
                action_append('turn004L')
                action_append('xialouti')
                time.sleep(1)
                action_append('turn004L')
                action_append('Left02move')
                action_append('Left02move')
                action_append('turn004L')
                # stage_position = stagePosition_Chest(Chest)
                # if (stage_position < 220) & (stage_position > 190):
                #     action_append('Left02move')
                # elif stage_position <= 190:
                #     action_append('Left02move')
                #     action_append('Left02move')
                # elif (stage_position > 260) & (stage_position < 290):
                #     action_append('Right02move')
                # elif stage_position >= 290:
                #     action_append('Right02move')
                #     action_append('Right02move')
                # else:
                #     pass
                action_append('Forwalk02')
                action_append('Forwalk02')
                action_append('Forwalk02')
                action_append('Forwalk02')
                action_append('Forwalk02')
                stage_clear = 1
            else:
                action_append('Forwalk02')
                num = num + 1
        print('stage clear')
        break
