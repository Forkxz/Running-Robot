import cv2
import numpy as np
from Color_define import *
from Landmine import getAreaMaxContour1
# from Robot_control import action_append
from CMDcontrol import action_append,action_list
import math

# 映射函数，缩小后的图片处理后得到的坐标，再放大得到原图所对应的点
def leMap_lza(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


########得到最大轮廓 姜颜曹所用############
def getAreaMaxContour2_lza(contours, area=1):
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > area:  # 面积大于1
                area_max_contour = c
    return area_max_contour


#######bffle函数#########
def baffle(Chest, resize_width, resize_height, baffleturn):
    global baffle_step

    ori_width = int(3 * 160)  # 原始图像640x480
    ori_height = int(4 * 160)

    # cv2.imshow('frame2',frame2)
    OrgFrame1 = Chest.imgs.copy()
    # OrgFrame1 = cv2.imread('0.jpg')
    # cv2.imshow('read',OrgFrame1)

    (h, w) = OrgFrame1.shape[:2]
    center = (w // 2, h // 2)
    R = cv2.getRotationMatrix2D(center, 90, 1.0)
    OrgFrame = cv2.warpAffine(OrgFrame1, R, (h, w))
    cv2.imshow("Rotated by -90 Degrees", OrgFrame)

    frame_Chest = cv2.resize(OrgFrame, (resize_width, resize_height), interpolation=cv2.INTER_LINEAR)
    cv2.imshow('init', frame_Chest)

    center = []
    # 开始处理图像
    hsv = cv2.cvtColor(frame_Chest, cv2.COLOR_BGR2HSV)  # rgv转hsv
    hsv = cv2.GaussianBlur(hsv, (3, 3), 0)  # 转完高斯滤波
    # cv2.imshow('hsv', hsv)
    Imask = cv2.inRange(hsv, color_dist_lza['flap']['Lower'], color_dist_lza['flap']['Upper'])
    # Imask = cv2.inRange(hsv, color_dist_lza['blue']['Lower'], color_dist_lza['blue']['Upper'])  # 提取颜色，此处为蓝
    # cv2.imshow('blue', Imask)
    Imask = cv2.erode(Imask, None, iterations=2)  # 腐蚀
    Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)  # 膨胀
    cv2.imshow('color', Imask)
    cv2.waitKey(1)
    _, cnts, hieracy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
    cnt_large = getAreaMaxContour2_lza(cnts, area=1000)  # 找出最大轮廓
    if cnt_large is None:
        print('no blue')
        action_append('Forwalk01')  # 向前走
        print('Forwalk01')
        return
    cnt_large_area = math.fabs(cv2.contourArea(cnt_large))  # 计算最大轮廓面积
    print('the largest area : ', cnt_large_area)

    if cnt_large_area > 4000:  # 判断进入挡板关卡，执行下面命令
        print('get ready for baffle')

        rect = cv2.minAreaRect(cnt_large)  # 最小外接矩形，存在
        # cv2.imshow('lunkuo', rect)
        box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
        box[0, 0] = int(leMap_lza(box[0, 0], 0, resize_width, 0, ori_width))  # 缩放图片，转换四个点的八个坐标值
        box[0, 1] = int(leMap_lza(box[0, 1], 0, resize_height, 0, ori_height))
        box[1, 0] = int(leMap_lza(box[1, 0], 0, resize_width, 0, ori_width))
        box[1, 1] = int(leMap_lza(box[1, 1], 0, resize_height, 0, ori_height))
        box[2, 0] = int(leMap_lza(box[2, 0], 0, resize_width, 0, ori_width))
        box[2, 1] = int(leMap_lza(box[2, 1], 0, resize_height, 0, ori_height))
        box[3, 0] = int(leMap_lza(box[3, 0], 0, resize_width, 0, ori_width))
        box[3, 1] = int(leMap_lza(box[3, 1], 0, resize_height, 0, ori_height))
        pt1_x, pt1_y = box[0, 0], box[0, 1]
        pt3_x, pt3_y = box[2, 0], box[2, 1]
        center_x = int((pt1_x + pt3_x) / 2)
        center_y = int((pt1_y + pt3_y) / 2)
        center.append([center_x, center_y])  # 矩形上边界中点添加到center列表里
        cv2.drawContours(OrgFrame, [box], -1, [0, 0, 255, 255], 3)
        cv2.circle(OrgFrame, (center_x, center_y), 10, (0, 0, 255), -1)  # 画出中心

        # 求得大矩形的旋转角度，if条件是为了判断长的一条边的旋转角度，因为box存储的点的顺序不确定\
        if math.sqrt(math.pow(box[3, 1] - box[0, 1], 2) + math.pow(box[3, 0] - box[0, 0], 2)) > math.sqrt(
                math.pow(box[3, 1] - box[2, 1], 2) + math.pow(box[3, 0] - box[2, 0], 2)):
            baffle_angle = - math.atan((box[3, 1] - box[0, 1]) / (box[3, 0] - box[0, 0])) * 180.0 / math.pi
        else:
            baffle_angle = - math.atan(
                (box[3, 1] - box[2, 1]) / (box[3, 0] - box[2, 0])) * 180.0 / math.pi  # 负号是因为坐标原点的问题

        # 判断方向是否正直
        print('angle: ', baffle_angle)
        if baffle_angle > 5:
            action_append('turn001L')  # 转向
            print('turn left')
            return
        elif baffle_angle < -5:
            action_append('turn001R')  # 转向
            print('turn right')
            return
        else:
            print('direction correct')

        # 以外接矩形的中点横坐标来校准位置
        print('center_x = ')
        print(center_x)
        if center_x > 288:
            action_append('Right02move')
            print('move right')
            return
        elif center_x < 268:
            action_append('Left02move')
            print('move left')
            return
        else:
            print('location correct')

        # 位置已校准，方向已对准，开始前进准备翻滚

        # 获取roi区域图像处理判断面积
        baffle_area_flag = False
        frame2 = frame_Chest[80:240, 120:320]  # 我的roi
        cv2.imshow('roi', frame2)
        center = []
        # 开始处理图像
        hsv2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)  # rgv转hsv
        hsv2 = cv2.GaussianBlur(hsv2, (3, 3), 0)  # 转完高斯滤波
        # cv2.imshow('hsv2', hsv2)
        Imask2 = cv2.inRange(hsv2, color_dist_lza['flap']['Lower'], color_dist_lza['flap']['Upper'])  # 提取颜色，此处为蓝
        # cv2.imshow('blue2', Imask2)
        Imask2 = cv2.erode(Imask2, None, iterations=2)  # 腐蚀
        Imask2 = cv2.dilate(Imask2, np.ones((3, 3), np.uint8), iterations=2)  # 膨胀
        cv2.imshow('color2', Imask2)
        _, cnts2, hieracy = cv2.findContours(Imask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
        cnt_large2 = getAreaMaxContour2_lza(cnts2, area=1000)  # 找出最大轮廓
        if cnt_large2 is None:
            print('no blue')
            action_append('Forwalk01')  # 向前走
            print('Forwalk01')
            return
        cnt_large_area2 = math.fabs(cv2.contourArea(cnt_large2))  # 计算最大轮廓面积
        print('the largest area2 : ', cnt_large_area2)
        if cnt_large_area2 > 14000:  # 判断roi区域做出动作
            action_append('Forwalk01')

            baffle_area_flag = True
            action_append('RollRail')
            print('roll now ！')
            if baffleturn:
                action_append('turn004L')
                action_append('turn004L')
                action_append('Left3move')
                action_append('turn004L')
                action_append('Left3move')
            else:
                action_append('turn004R')
                action_append('turn004R')
                action_append('turn004R')
                action_append('Right3move')

            baffle_step = 1

        else:
            action_append('Forwalk01')
            print('go forwalk')

        cv2.putText(OrgFrame, "area:" + str(cnt_large_area),
                    (10, OrgFrame.shape[0] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

        cv2.putText(OrgFrame, "are_flag:" + str(baffle_area_flag),
                    (10, OrgFrame.shape[0] - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

        cv2.putText(OrgFrame, "angle:" + str(baffle_angle),
                    (10, OrgFrame.shape[0] - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

        cv2.putText(OrgFrame, "center_x:" + str(center_x),
                    (10, OrgFrame.shape[0] - 95), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

        cv2.imshow('OrgFrame', OrgFrame)


    else:
        action_append('Forwalk01')  # 向前走
        print('Forwalk01')

def baffle_function_lza(Chest,baffleturn):
    global baffle_step
    baffle_step = 0
    while True:
        if baffle_step == 0:
            baffle(Chest, 320, 240, baffleturn)
            print('Running')
        elif baffle_step == 1:
            print('goodbye to baffle')
            cv2.destroyAllWindows()
            break