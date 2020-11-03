import cv2
from Color_define import *
from Landmine import getAreaMaxContour1
import time
from CMDcontrol import action_append
# from Robot_control import action_append

def end_door(Chest):
    while True:
        t1 = cv2.getTickCount() # 时间计算

        handling = Chest.imgs
        cv2.imshow('handling', handling)
        cv2.waitKey(1)

        border = cv2.copyMakeBorder(handling, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,value=(255, 255, 255))     # 扩展白边，防止边界无法识别
        handling = cv2.resize(border, (DIM[0], DIM[1]), interpolation=cv2.INTER_CUBIC)                   # 将图片缩放
        frame_gauss = cv2.GaussianBlur(handling, (21, 21), 0)       # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)    # 将图片转换到HSV空间

        frame_door_yellow = cv2.inRange(frame_hsv, color_range['yellow_door'][0], color_range['yellow_door'][1])    # 对原图像和掩模(颜色的字典)进行位运算
        frame_door_black = cv2.inRange(frame_hsv, color_range['black_door'][0], color_range['black_door'][1])       # 对原图像和掩模(颜色的字典)进行位运算

        frame_door = cv2.add(frame_door_yellow, frame_door_black)    
        open_pic = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))      # 开运算 去噪点
        closed_pic = cv2.morphologyEx(open_pic, cv2.MORPH_CLOSE, np.ones((50, 50), np.uint8))   # 闭运算 封闭连接

        (image, contours, hierarchy) = cv2.findContours(closed_pic, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour, area_max = getAreaMaxContour1(contours)  # 找出最大轮廓
        percent = round(100 * area_max / (DIM[0] * DIM[1]), 2)  # 最大轮廓的百分比

        # 根据比例得到是否前进的信息
        if percent > 5:    #检测到横杆
            print(percent,"%")
            print("有障碍 等待 contours len：",len(contours))
            time.sleep(0.01)
        else:
            print(percent)
            print("执行快走04-快走9步")
            action_append("fastForward04")
            print('Finish End Door')
            break

