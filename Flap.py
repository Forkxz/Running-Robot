import cv2
from Color_define import *
from Landmine import getAreaMaxContour1
# from Robot_control import action_append
from CMDcontrol import action_append
import math

#########################################一些子函数##################################################################
# 映射函数，缩小后的图片处理后得到的坐标，再放大得到原图所对应的点
ori_width = int(3 * 160)  # 原始图像640x480
ori_height = int(4 * 160)
r_width = int(4 * 20)  # 处理图像时缩小为80x60,加快处理速度，谨慎修改！
r_height = int(3 * 20)
def leMap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

########得到最大轮廓 姜颜曹所用############
def getAreaMaxContour2(contours, area=1):
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > area:  # 面积大于1
                area_max_contour = c
    return area_max_contour

def baffle(Input_img, baffle_step, resize_width, resize_height):

            OrgFrame1 = Input_img
           # OrgFrame1 = cv2.imread('0.jpg')
            #cv2.imshow('read',OrgFrame1)

            (h, w) = OrgFrame1.shape[:2]
            center = (w // 2, h // 2)
            R = cv2.getRotationMatrix2D(center, 90, 1.0)
            OrgFrame = cv2.warpAffine(OrgFrame1, R, (h, w))
            cv2.imshow("Rotated by -90 Degrees", OrgFrame)

            frame = cv2.resize(OrgFrame, (resize_width, resize_height), interpolation=cv2.INTER_LINEAR)
            cv2.imshow('init', frame)

            center = []
            # 开始处理图像
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # rgv转hsv
            hsv = cv2.GaussianBlur(hsv, (3, 3), 0)  # 转完高斯滤波
            #cv2.imshow('hsv', hsv)
            Imask = cv2.inRange(hsv, color_dist['flap']['Lower'], color_dist['flap']['Upper'])
            #Imask = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])  # 提取颜色，此处为蓝
            #cv2.imshow('blue', Imask)
            Imask = cv2.erode(Imask, None, iterations=2)  # 腐蚀
            Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)  # 膨胀
            cv2.imshow('color', Imask)
            cv2.waitKey(1)
            _, cnts, hieracy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
            cnt_large = getAreaMaxContour2(cnts, area=1000)  # 找出最大轮廓
            if cnt_large is None:
                print('no blue')
                action_append('Forwalk00')  # 向前走
                print('Forwalk00')
                return
            cnt_large_area = math.fabs(cv2.contourArea(cnt_large)) #计算最大轮廓面积
            print('the largest area : ', cnt_large_area)

            if cnt_large_area > 4000:                       #判断进入挡板关卡，执行下面命令
                print('get ready for baffle')

                rect = cv2.minAreaRect(cnt_large)  # 最小外接矩形，存在
                #cv2.imshow('lunkuo', rect)
                box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
                box[0, 0] = int(leMap(box[0, 0], 0, resize_width, 0, ori_width))  # 缩放图片，转换四个点的八个坐标值
                box[0, 1] = int(leMap(box[0, 1], 0, resize_height, 0, ori_height))
                box[1, 0] = int(leMap(box[1, 0], 0, resize_width, 0, ori_width))
                box[1, 1] = int(leMap(box[1, 1], 0, resize_height, 0, ori_height))
                box[2, 0] = int(leMap(box[2, 0], 0, resize_width, 0, ori_width))
                box[2, 1] = int(leMap(box[2, 1], 0, resize_height, 0, ori_height))
                box[3, 0] = int(leMap(box[3, 0], 0, resize_width, 0, ori_width))
                box[3, 1] = int(leMap(box[3, 1], 0, resize_height, 0, ori_height))
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

                #以外接矩形的中点横坐标来校准位置
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



                #位置已校准，方向已对准，开始前进准备翻滚

                #获取roi区域图像处理判断面积
                baffle_area_flag = False
                frame2 = frame[80:240, 120:320]          # 我的roi
                cv2.imshow('roi', frame2)
                center = []
                # 开始处理图像
                hsv2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)  # rgv转hsv
                hsv2 = cv2.GaussianBlur(hsv2, (3, 3), 0)  # 转完高斯滤波
                #cv2.imshow('hsv2', hsv2)
                Imask2 = cv2.inRange(hsv2, color_dist['blue']['Lower'], color_dist['blue']['Upper'])  # 提取颜色，此处为蓝
                #cv2.imshow('blue2', Imask2)
                Imask2 = cv2.erode(Imask2, None, iterations=2)  # 腐蚀
                Imask2 = cv2.dilate(Imask2, np.ones((3, 3), np.uint8), iterations=2)  # 膨胀
                cv2.imshow('color2', Imask2)
                _, cnts2, hieracy = cv2.findContours(Imask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
                cnt_large2 = getAreaMaxContour2(cnts2, area=1000)  # 找出最大轮廓
                if cnt_large2 is None:
                    print('no blue')
                    action_append('Forwalk00')  # 向前走
                    print('Forwalk00')
                    return
                cnt_large_area2 = math.fabs(cv2.contourArea(cnt_large2))  # 计算最大轮廓面积
                print('the largest area2 : ', cnt_large_area2)
                if cnt_large_area2 > 12000:  # 判断roi区域做出动作
                    action_append('Forwalk00')

                    baffle_area_flag = True
                    action_append('RollRail')
                    print('roll now ！')

                    for i in range(9):
                        action_append('turn004L')
                        print('turnL')

                    baffle_step = 1

                else:
                    action_append('Forwalk00')
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
                action_append('Forwalk00')  # 向前走
                print('Forwalk00')


def afterbaffle_turn(Input_img, baffle_step, resize_width, resize_height):
    OrgFrame1 = Input_img
    # OrgFrame1 = cv2.imread('0.jpg')
    # cv2.imshow('read',OrgFrame1)
    (h, w) = OrgFrame1.shape[:2]
    center = (w // 2, h // 2)
    R = cv2.getRotationMatrix2D(center, 90, 1.0)
    OrgFrame = cv2.warpAffine(OrgFrame1, R, (h, w))
    cv2.imshow("after Rotated by -90 Degrees", OrgFrame)
    frame = cv2.resize(OrgFrame, (resize_width, resize_height), interpolation=cv2.INTER_LINEAR)
    cv2.imshow('after init', frame)
    center = []
    # 开始处理图像
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # rgv转hsv
    hsv = cv2.GaussianBlur(hsv, (3, 3), 0)  # 转完高斯滤波
    cv2.imshow('after hsv', hsv)
    Imask = cv2.inRange(hsv, color_dist['flap']['Lower'], color_dist['flap']['Upper'])
    # Imask = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])  # 提取颜色，此处为蓝
    cv2.imshow('after blue', Imask)
    Imask = cv2.erode(Imask, None, iterations=2)  # 腐蚀
    Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)  # 膨胀
    cv2.imshow('after color', Imask)
    _, cnts, hieracy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
    cnt_large = getAreaMaxContour2(cnts, area=500)  # 找出最大轮廓
    if cnt_large is None:
                print('no blue')
                action_append('Forwalk01')  # 向前走
                print('Forwalk4cm')
                return
    rect = cv2.minAreaRect(cnt_large)  # 最小外接矩形，存在

    box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
    box[0, 0] = int(leMap(box[0, 0], 0, resize_width, 0, ori_width))  # 缩放图片，转换四个点的八个坐标值
    box[0, 1] = int(leMap(box[0, 1], 0, resize_height, 0, ori_height))
    box[1, 0] = int(leMap(box[1, 0], 0, resize_width, 0, ori_width))
    box[1, 1] = int(leMap(box[1, 1], 0, resize_height, 0, ori_height))
    box[2, 0] = int(leMap(box[2, 0], 0, resize_width, 0, ori_width))
    box[2, 1] = int(leMap(box[2, 1], 0, resize_height, 0, ori_height))
    box[3, 0] = int(leMap(box[3, 0], 0, resize_width, 0, ori_width))
    box[3, 1] = int(leMap(box[3, 1], 0, resize_height, 0, ori_height))
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
            action_append('Forwalk02')
            action_append('Forwalk01')
            for i in range(5):
                action_append('turn004R')
            baffle_step = 2
            print('finish')

def afterbaffle_straight(resize_width, resize_height):

    global baffle_step
    global frame


    #ret, frame = cap_chest.read()
    # cv2.imshow('frame2',frame2)
    OrgFrame1 = frame
    # OrgFrame1 = cv2.imread('0.jpg')
    # cv2.imshow('read',OrgFrame1)
    (h, w) = OrgFrame1.shape[:2]
    center = (w // 2, h // 2)
    R = cv2.getRotationMatrix2D(center, 90, 1.0)
    OrgFrame = cv2.warpAffine(OrgFrame1, R, (h, w))
    cv2.imshow("after Rotated by -90 Degrees", OrgFrame)
    frame = cv2.resize(OrgFrame, (resize_width, resize_height), interpolation=cv2.INTER_LINEAR)
    cv2.imshow('after init', frame)
    center = []
    # 开始处理图像
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # rgv转hsv
    hsv = cv2.GaussianBlur(hsv, (3, 3), 0)  # 转完高斯滤波
    cv2.imshow('after hsv', hsv)
    Imask = cv2.inRange(hsv, color_dist['flap']['Lower'], color_dist['flap']['Upper'])
    # Imask = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])  # 提取颜色，此处为蓝
    cv2.imshow('after blue', Imask)
    Imask = cv2.erode(Imask, None, iterations=2)  # 腐蚀
    Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)  # 膨胀
    cv2.imshow('after color', Imask)
    _, cnts, hieracy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
    cnt_large = getAreaMaxContour2(cnts, area=500)  # 找出最大轮廓
    if cnt_large is None:
                print('no blue')
                action_append('Forwalk01')  # 向前走
                print('Forwalk4cm')
                return
    rect = cv2.minAreaRect(cnt_large)  # 最小外接矩形，存在

    box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
    box[0, 0] = int(leMap(box[0, 0], 0, resize_width, 0, ori_width))  # 缩放图片，转换四个点的八个坐标值
    box[0, 1] = int(leMap(box[0, 1], 0, resize_height, 0, ori_height))
    box[1, 0] = int(leMap(box[1, 0], 0, resize_width, 0, ori_width))
    box[1, 1] = int(leMap(box[1, 1], 0, resize_height, 0, ori_height))
    box[2, 0] = int(leMap(box[2, 0], 0, resize_width, 0, ori_width))
    box[2, 1] = int(leMap(box[2, 1], 0, resize_height, 0, ori_height))
    box[3, 0] = int(leMap(box[3, 0], 0, resize_width, 0, ori_width))
    box[3, 1] = int(leMap(box[3, 1], 0, resize_height, 0, ori_height))
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
            action_append('Forwalk02')
            action_append('Forwalk01')
            for i in range(9):
                action_append('turn004R')
            baffle_step = 2
            print('finish')

def baffle_function_lza(Input_img):
    baffle_step = 0
    if baffle_step == 0:
        baffle(Input_img, baffle_step, 320, 240)
        print('Running')
    elif baffle_step == 1:
        print('correcting')
        afterbaffle_turn(320, 240)
    else:
        print('goodbye to baffle')

def flap(ChestOrg_img):

    org_img_copy = np.rot90(ChestOrg_img)[400:640, 80:400].copy()
    handling = org_img_copy.copy()
    frame_gauss = cv2.GaussianBlur(handling, (21, 21), 0)       # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)    # 将图片转换到HSV空间
    
    #h,s,v = cv2.split(frame_hsv)
    #cv2.imshow("s space", s)
    #cv2.waitKey(1)

    frame_flap = cv2.inRange(frame_hsv, color_dist['flap']['Lower'], color_dist['flap']['Upper'])
    
    open_pic = cv2.morphologyEx(frame_flap, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))      # 开运算 去噪点
    closed_pic = cv2.morphologyEx(open_pic, cv2.MORPH_CLOSE, np.ones((50, 50), np.uint8))   # 闭运算 封闭连接

    contours, _ = cv2.findContours(closed_pic, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
    areaMaxContour, area_max = getAreaMaxContour1(contours)  # 找出最大轮廓
    percent = round(100 * area_max / (DIM[0] * DIM[1]), 2)  # 最大轮廓的百分比

    print(percent,"%")

    if areaMaxContour is not None:
        rect = cv2.minAreaRect(areaMaxContour)  # 矩形框选
        box = np.int0(cv2.boxPoints(rect))      # 点的坐标
        cv2.drawContours(handling, [box], 0, (153, 200, 0), 2)  # 将最小外接矩形画在图上
        cv2.imshow("flap test", handling)
        cv2.waitKey(1)
        
    if percent > 12:
        print("roll")
        #action_append("RollRail")
        event_state = "finish_event"
    else:
        action_append("Forwalk00")
    return event_state

if __name__ == '__main__':
    
    Chest_path = '../track_picture/test/1005chest5.png'
    Chest_img = cv2.imread(Chest_path,1)
    Chest_img = cv2.resize(Chest_img,(640,480))
    flap(Chest_img)