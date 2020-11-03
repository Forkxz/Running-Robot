import cv2
import math
import numpy as np
import threading
import time
import datetime

from Color_define import *
from CMDcontrol import action_append, action_list
# from Robot_control import action_append,action_list
###############得到线形的总的轮廓###############
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

########得到最大轮廓############
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

######### 得到所有轮廓的面积##########
def getAreaSumContour(contours):
    contour_area_sum = 0
    for c in contours:  # 历遍所有轮廓
        contour_area_sum += math.fabs(cv2.contourArea(c))  # 计算轮廓面积
    return contour_area_sum  # 返回最大的面积

def start_door(Chest):
    waited = 0
    event_step_startdoor = 0

    while True:
        t1 = cv2.getTickCount() # 时间计算
        # cv2.imshow("ChestOrg_img", Chest.imgs)#[0])
        #cv2.waitKey(1)
        org_img_copy = np.rot90(Chest.imgs).copy()
        handling = org_img_copy.copy()[100:250]
#         cv2.imshow("HeadOrg_img", Head.imgs)
#         cv2.waitKey(1)
#         org_img_copy = Head.imgs.copy()
#         handling = org_img_copy.copy()[150:350, :]
        # handling = cv2.imread("D:\\akejian\\runningrobot\\videos\\3.PNG")
        # handling = handling[150:350, :]

        border = cv2.copyMakeBorder(handling, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,value=(255, 255, 255))     # 扩展白边，防止边界无法识别
        handling = cv2.resize(border, (DIM[0], DIM[1]), interpolation=cv2.INTER_CUBIC)                   # 将图片缩放
        
        cv2.imshow("handling", handling)
        cv2.waitKey(1)
        
        frame_gauss = cv2.GaussianBlur(handling, (21, 21), 0)       # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)    # 将图片转换到HSV空间

        frame_door_yellow = cv2.inRange(frame_hsv, color_range['yellow_door'][0], color_range['yellow_door'][1])    # 对原图像和掩模(颜色的字典)进行位运算
        frame_door_black = cv2.inRange(frame_hsv, color_range['black_door_new'][0], color_range['black_door_new'][1])       # 对原图像和掩模(颜色的字典)进行位运算

        #frame_door = cv2.add(frame_door_yellow, frame_door_black)    
        open_pic_yellow = cv2.morphologyEx(frame_door_yellow, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))      # 开运算 去噪点
        closed_pic_yellow = cv2.morphologyEx(open_pic_yellow, cv2.MORPH_CLOSE, np.ones((50, 50), np.uint8))   # 闭运算 封闭连接
        (image_yellow, contours_yellow, hierarchy_yellow) = cv2.findContours(closed_pic_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour_yellow, area_max_yellow = getAreaMaxContour1(contours_yellow)  # 找出最大轮廓
        percent_yellow = round(100 * area_max_yellow / (DIM[0] * DIM[1]), 2)  # 最大轮廓的百分比
        
        open_pic_black = cv2.morphologyEx(frame_door_black, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))      # 开运算 去噪点
        closed_pic_black = cv2.morphologyEx(open_pic_black, cv2.MORPH_CLOSE, np.ones((50, 50), np.uint8))   # 闭运算 封闭连接
        (image_black, contours_black, hierarchy_black) = cv2.findContours(closed_pic_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour_black, area_max_black = getAreaMaxContour1(contours_black)  # 找出最大轮廓
        percent_black = round(100 * area_max_black / (DIM[0] * DIM[1]), 2)  # 最大轮廓的百分比
        
        cv2.imshow("image_yellow", image_yellow)
        #cv2.imshow("image_black", image_black)
        cv2.waitKey(1)

        # 根据比例得到是否前进的信息
        if event_step_startdoor == 1:
            if percent_yellow < 2:
                event_step_startdoor = 2
                break
            else:
                event_step_startdoor = 0
                action_append("Back2Run")
                while len(action_list) != 0:
                    time.sleep(0.1)
        else:
            if percent_yellow > 2:    #检测到横杆
                print(percent_yellow,"%")
                print("有障碍 等待")
                waited = 1
                time.sleep(0.01)
            else:
                print(percent_yellow,"%")
                if waited == 1:
                    print("执行快走05-快走12步")
                    action_append("fastForward06")
                    while len(action_list) != 0:
                        time.sleep(0.1)
                    event_step_startdoor = 1
                    cv2.destroyAllWindows()
                else:
                    print("继续等吧，如果还没走过去横杆就下来了怎么办")

        
if __name__ == '__main__':
    print('This is start_door_state')
    start_door()