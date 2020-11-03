import cv2
import numpy as np
import time
from Color_define import *
from Landmine import getAreaMaxContour1
# from Robot_control import action_append,action_list
from CMDcontrol import action_append,action_list
def step(Chest):
    while True:
        org_img_copy = np.rot90(Chest.imgs)

        border = cv2.copyMakeBorder(org_img_copy, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,value=(255, 255, 255))     # 扩展白边，防止边界无法识别
        handling = cv2.resize(border, (DIM[0], DIM[1]), interpolation=cv2.INTER_CUBIC)                   # 将图片缩放
        frame_gauss = cv2.GaussianBlur(handling, (21, 21), 0)       # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)    # 将图片转换到HSV空间

        frame_door_rgb = cv2.inRange(frame_hsv, color_range['red blue green'][0], color_range['red blue green'][1])    # 对原图像和掩模(颜色的字典)进行位运算  
        open_pic = cv2.morphologyEx(frame_door_rgb, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))      # 开运算 去噪点
        closed_pic = cv2.morphologyEx(open_pic, cv2.MORPH_CLOSE, np.ones((50, 50), np.uint8))   # 闭运算 封闭连接

        contours = cv2.findContours(closed_pic, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour, area_max = getAreaMaxContour1(contours)  # 找出最大轮廓
        M = cv2.moments(areaMaxContour)
        cX = int(M["m10"] / (M["m00"] + 0.0001))
        cY = int(M["m01"] / (M["m00"] + 0.0001))
        cv2.drawContours(closed_pic, contours, -1, (0, 255, 0), 2)
        cv2.imshow("image", closed_pic)
        cv2.waitKey(1)
        percent = round(100 * area_max / (DIM[0] * DIM[1]), 2)  # 最大轮廓的百分比

        if cY > 190:
            if y_dis > 50:
                if cX > 140:
                    print("Right3move")
                    action_append("Right3move")
                    while len(action_list) != 0:
                        time.sleep(0.1)
                else:
                    print("Right02move")
                    action_append("Right02move")
                    while len(action_list) != 0:
                        time.sleep(0.1)                      
            elif y_dis > 20:
                if cX > 300:
                    print("Right3move")
                    action_append("Right3move")
                    while len(action_list) != 0:
                        time.sleep(0.1)
                else:
                    print("Right02move")
                    action_append("Right02move")
                    while len(action_list) != 0:
                        time.sleep(0.1)
            else:
                if cY != 0 and cX <= 170:
                    event_step_bridge = 1
                    print("event_step_bridge =", event_step_bridge)
        elif cY < 190:
            if y_dis > 50 and cY != 0:
                if cX > 140:
                    print("Left3move")
                    action_append("Left3move")
                    while len(action_list) != 0:
                        time.sleep(0.1)
                else:
                    print("Left02move")
                    action_append("Left02move")
                    while len(action_list) != 0:
                        time.sleep(0.1) 
            elif y_dis > 20 and cY != 0:
                if cX > 300:
                    print("Left3move")
                    action_append("Left3move")
                    while len(action_list) != 0:
                        time.sleep(0.1)
                else:
                    print("Left02move")
                    action_append("Left02move")
                    while len(action_list) != 0:
                        time.sleep(0.1)
            else:
                if cY != 0 and cX <= 170:
                    event_step_bridge = 1

        if cX == 0 and cY == 0:
            action_append("Forwalk02")

        # 根据比例得到是否前进的信息
        if percent > 15:    #检测到横杆
            print(percent,"%")
            print("快走",len(contours))
            action_append("fastForward04")
            action_append("UpBridge")
            action_append("DownBridge")
            break
        else:
            print(percent,"%")
            action_append("fastForward04")

if __name__ == '__main__':
    
    Chest_path = '../track_picture/test/1005chest5.png'
    Chest_img = cv2.imread(Chest_path,1)
    Chest_img = cv2.resize(Chest_img,(640,480))
    step(Chest_img)