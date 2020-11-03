import cv2
import numpy as np
import time
# from Robot_control import action_append,action_list
from CMDcontrol import action_append, action_list
from Landmine import getAreaMaxContour1
from Center import group_lines_and_draw, Calculate_position
from Undistort import undistort_chest


def holeb(Chest):
    counter_forwalk = 0
    event_step = 0
    while True:
        if event_step == 0:
            frame = undistort_chest(Chest.imgs)
            img = frame[30:450, 50:400]
            img_test = img.copy()
            fsrc = np.array(img, dtype=np.float32) / 255.0
            (b, g, r) = cv2.split(fsrc)
            #lv
            #gray = 2 * g - b - r
            gray = 2*b - g- r
            (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
            hist = cv2.calcHist([gray], [0], None, [256], [minVal, maxVal])
            gray_u8 = np.array((gray - minVal) / (maxVal - minVal) * 255, dtype=np.uint8)
            (t, thresh_img) = cv2.threshold(gray_u8, -1.0, 255, cv2.THRESH_OTSU+cv2.THRESH_BINARY_INV)
            cv2.imshow("th", thresh_img)
            cv2.imshow("th", thresh_img)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
            thresh = cv2.morphologyEx(thresh_img, cv2.MORPH_OPEN, kernel)
            # edges = cv2.Canny(thresh,20,100,apertureSize = 3,L2gradient=0)
            _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE,
                                              cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, contours, -1, (0, 255, 0), 2)
            #cv2.imshow('c',img)
            #cv2.waitKey(1)
            length = len(contours)
            list1 = []
            flag_hole = False
            for i in range(length):
                cnt = contours[i]
                #epsilon = 0.02*cv2.arcLength(cnt,True)
                approx = cv2.approxPolyDP(cnt, 15, True)
                if (({len(approx) == 4 or len(approx) == 5})
                        and (cv2.isContourConvex(approx))):
                    list1.append(approx)
                    flag_hole = True
            if flag_hole == True:
                area_max_contour, _ = getAreaMaxContour1(list1)
                M = cv2.moments(area_max_contour)
                cX = int(M["m10"] / (M["m00"] + 0.0001))
                cY = int(M["m01"] / (M["m00"] + 0.0001))
                # midpoint=[cX,cY]
                print("cX is {} and cY is {}".format(cX, cY))

                cv2.circle(img_test, (cX, cY), 6, 1, -1)
                cv2.drawContours(img_test, area_max_contour, -1, (0, 0, 255),
                                 2)
                cv2.imshow("hole test", img_test)
                cv2.waitKey(1)

                x_dis = abs(cX - 150)
                y_dis = abs(cY - 190)
                # y_right = abs(300 - cY)
                # y_left = abs(220 - cY)
                print('x_dis =', x_dis)

                if cX > 150:
                    if x_dis > 50:
                        action_append("Forwalk02")
                        action_append("turn000R")
                        action_append("HeadTurnMM")
                        while len(action_list) != 0:
                            time.sleep(0.1)
                    elif x_dis > 30:
                        action_append("Forwalk01")
                        action_append("turn000R")
                        action_append("HeadTurnMM")
                        while len(action_list) != 0:
                            time.sleep(0.1)
                    elif x_dis > 20:
                        action_append("Forwalk00")
                        action_append("turn000R")
                        action_append("HeadTurnMM")
                        while len(action_list) != 0:
                            time.sleep(0.1)
                    else:
                        event_step = 1
                elif cX > 130 and cX < 170:
                    event_step = 1
                else:
                    action_append("Back2Run")
                    action_append("HeadTurnMM")
                    while len(action_list) != 0:
                        time.sleep(0.1)

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

                # cv2.circle(img_test, (cX, cY), 6, 1, -1)
                # cv2.drawContours(img_test, area_max_contour, -1, (0, 0, 255), 2)
                # cv2.imshow("hole test", img_test)
                # cv2.waitKey(1)
            else:
                action_append("Right02move")
                action_append("HeadTurnMM")
                while len(action_list) != 0:
                    time.sleep(0.1)

        if event_step == 1:
            action_append("turn001R")
            action_append("turn001R")
            action_append("Right3move")
            action_append("Right3move")
            action_append("Right3move")
            #action_append("Right3move")
            action_append("HeadTurnMM")
            while len(action_list) != 0:
                time.sleep(0.1)
            event_step = 2

        if event_step == 2:
            ROI_image = np.rot90(Chest.imgs).copy()[250:550, :]
            cv2.imshow("Chest_img", ROI_image)
            cv2.waitKey(1)

            Canny_img = cv2.Canny(ROI_image, 15, 150)
            # cv2.imshow("Canny_img",Canny_img)
            # cv2.waitKey(1)

            #膨胀加粗边缘
            dilate = cv2.dilate(Canny_img,
                                np.ones((2, 2), np.uint8),
                                iterations=1)
            cv2.imshow("dilate", dilate)
            cv2.waitKey(1)

            print('event2')

            Lines = cv2.HoughLinesP(dilate,
                                    1.0,
                                    np.pi / 180,
                                    100,
                                    minLineLength=50,
                                    maxLineGap=10)
            final_image, right_side, _ = group_lines_and_draw(
                ROI_image, Lines, 'Right')
            if right_side is None:
                action_append("Forwalk01")
                action_append("turn001R")
                continue
            dx, deg = Calculate_position(right_side)
            print('right', dx, deg)

            if dx > 10:
                # print("Right3move")
                action_append("Right3move")
                action_append("HeadTurnMM")
                while len(action_list) != 0:
                    time.sleep(0.1)
                event_step = 3
            else:
                event_step = 3

        if event_step == 3:
            ROI_image = np.rot90(Chest.imgs).copy()[250:550, :]
            cv2.imshow("Chest_img", ROI_image)
            #cv2.waitKey(1)

            Canny_img = cv2.Canny(ROI_image, 15, 150)
            # cv2.imshow("Canny_img",Canny_img)
            # cv2.waitKey(1)

            #膨胀加粗边缘
            dilate = cv2.dilate(Canny_img,
                                np.ones((2, 2), np.uint8),
                                iterations=1)
            #cv2.imshow("dilate",dilate)
            #cv2.waitKey(1)

            print('event3')

            Lines = cv2.HoughLinesP(dilate,
                                    1.0,
                                    np.pi / 180,
                                    100,
                                    minLineLength=50,
                                    maxLineGap=10)
            final_image, right_side, _ = group_lines_and_draw(
                ROI_image, Lines, 'Right')
            if right_side is None:
                action_append("Forwalk01")
                action_append("turn001R")
                continue
            dx, deg = Calculate_position(right_side)
            print('right', dx, deg)
            cv2.imshow("image line", final_image)
            cv2.waitKey(1)

            if deg >= 7:
                print("turn001L")
                action_append("turn001L")
                action_append("HeadTurnMM")
                while len(action_list) != 0:
                    time.sleep(0.1)
            elif deg <= -7:
                print("turn001R")
                action_append("turn001R")
                action_append("HeadTurnMM")
                while len(action_list) != 0:
                    time.sleep(0.1)
            if dx >= 12:
                print("Right02move")
                action_append("Right02move")
                action_append("HeadTurnMM")
                while len(action_list) != 0:
                    time.sleep(0.1)
            elif dx <= 8:
                print("Left02move")
                action_append("Left02move")
                action_append("HeadTurnMM")
                while len(action_list) != 0:
                    time.sleep(0.1)

            if -7 < deg < 7 and 8 < dx < 12:
                print("go straight")
                action_append("Forwalk02")
                action_append("turn000R")
                action_append("Forwalk02")
                action_append("turn000R")
                action_append("HeadTurnMM")
                while len(action_list) != 0:
                    time.sleep(0.1)
                counter_forwalk = counter_forwalk + 1
                if counter_forwalk == 2:
                    event_step = 4

        if event_step == 4:
            ROI_image = np.rot90(Chest.imgs).copy()[250:550, :]
            cv2.imshow("Chest_img", ROI_image)
            cv2.waitKey(1)

            Canny_img = cv2.Canny(ROI_image, 15, 150)
            # cv2.imshow("Canny_img",Canny_img)
            # cv2.waitKey(1)

            #膨胀加粗边缘
            dilate = cv2.dilate(Canny_img,
                                np.ones((2, 2), np.uint8),
                                iterations=1)
            #cv2.imshow("dilate",dilate)
            cv2.waitKey(1)

            print('event_step == 4')

            Lines = cv2.HoughLinesP(dilate,
                                    1.0,
                                    np.pi / 180,
                                    100,
                                    minLineLength=50,
                                    maxLineGap=10)
            final_image, right_side, _ = group_lines_and_draw(
                ROI_image, Lines, 'Right')
            if right_side is None:
                action_append("Forwalk01")
                action_append("turn001R")
                continue
            dx, deg = Calculate_position(right_side)
            print('right', dx, deg)
            cv2.imshow("image line", final_image)
            #cv2.waitKey(1)

            if deg >= 7:
                print("turn001L")
                action_append("turn001L")
                action_append("HeadTurnMM")
                while len(action_list) != 0:
                    time.sleep(0.1)
            elif deg <= -7:
                print("turn001R")
                action_append("turn001R")
                action_append("HeadTurnMM")
                while len(action_list) != 0:
                    time.sleep(0.1)
            if dx >= 12:
                print("Right02move")
                action_append("Right02move")
                action_append("HeadTurnMM")
                while len(action_list) != 0:
                    time.sleep(0.1)
            elif dx <= 8:
                print("Left02move")
                action_append("Left02move")
                action_append("HeadTurnMM")
                while len(action_list) != 0:
                    time.sleep(0.1)

            if -7 < deg < 7 and 8 < dx < 12:
                print("go left")
                action_append("Left3move")
                action_append("Left3move")
                action_append("Left3move")
                action_append("HeadTurnMM")
                while len(action_list) != 0:
                    time.sleep(0.1)
                event_step = 5
        if event_step == 5:
            print('Finish Hole')
            cv2.destroyAllWindows()
            break


if __name__ == '__main__':

    Chest_path = '../track_picture/test/1005chest5.png'
    Chest_img = cv2.imread(Chest_path, 1)
    Chest_img = cv2.resize(Chest_img, (640, 480))
    # cv2.imshow("Chest_img",Chest_img)
    # cv2.waitKey(1)
    hole(Chest)