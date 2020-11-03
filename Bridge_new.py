import cv2
import numpy as np
import time

import math
from Undistort import undistort_chest
from Center import draw_lines,group_lines_and_draw
from Video_stream import LoadStreams
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
def greenRefine(src):
    # Extract green color with 2g - b - r
    start = time.time()
    ##Convert to yuv
    #src = cv2.cvtColor(src,cv2.COLOR_BGR2YUV)
    ##Hist_Equilzation
    #src[:,:,0] = cv2.equalizeHist(src[:,:,0])
    ##convert2BGR
    #src = cv2.cvtColor(src,cv2.COLOR_YUV2BGR)
    # Convert to float
    fsrc = np.array(src, dtype=np.float32) / 255.0
    (b, g, r) = cv2.split(fsrc)
    gray = 2 * g - b - r
    #Over Exposure
    #for i in range(0, g.shape[0]):
    #    for j in range(0, g.shape[1]):
    #        gray[i, j] = np.min
    # Get maximum and minimum value
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
    # Calculate Histogram
    hist = cv2.calcHist([gray], [0], None, [256], [minVal, maxVal])
    # Convert to u8, ostu threshold
    gray_u8 = np.array((gray - minVal) / (maxVal - minVal) * 255, dtype=np.uint8)
    (thresh, bin_img) = cv2.threshold(gray_u8, -1.0, 255, cv2.THRESH_OTSU)
    #cv2.imshow("bin_img", bin_img)
    #cv2.waitKey(1)
    ## Convert to colorful image
    #(b8, g8, r8) = cv2.split(src)
    #color_img = cv2.merge([b8 & bin_img, g8 & bin_img, r8 & bin_img])
    finish = time.time()
    timeConsume = finish - start
    #cv2.imshow("bin_img", bin_img)
    #cv2.waitKey(1)
    return bin_img

def bridge(Chest):
    event_step_bridge = 0
    while True:
        img = undistort_chest(Chest)
        img = img[50:430,50:400].copy()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


        height = img.shape[0]
        width = img.shape[1]
        blank_img = np.zeros((height,width,1), np.uint8)

        h,s,v = cv2.split(img)
        cv2.imshow("s", s)
        thresh_value,thresh_img = cv2.threshold(s, 110, 255, cv2.THRESH_BINARY)
        canny = cv2.Canny(thresh_img, 50, 150)
        cv2.imshow("c", canny)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(10,10))
        thresh = cv2.morphologyEx(thresh_img,cv2.MORPH_OPEN,kernel)
        #cv2.imshow("thresh", thresh)

        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        length = len(contours)
        list1 = []
        for i in range(length):
            cnt = contours[i]
            epsilon = 0.02*cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt, 20, True)
            #if(len(approx) == 4 or len(approx) == 5):
            list1.append(approx)
        area_max_contour, contour_area_max = getAreaMaxContour1(list1)
        M = cv2.moments(area_max_contour)
        cX = int(M["m10"] / (M["m00"] + 0.0001))
        cY = int(M["m01"] / (M["m00"] + 0.0001))
        cv2.polylines(blank_img, [area_max_contour], 1, (255, 0, 0), 1, cv2.LINE_4)

        # final_image, Final_line, good = group_lines_and_draw(
        # blank_img, Lines, wich_side)

        midpoint = [cX,cY]
        # cv2.polylines(blank_img, [area_max_contour], 1, (255, 0, 255), 1, cv2.LINE_4)
        cv2.circle(blank_img, (cX,cY), 6, (255,255,255), -1)
        cv2.imshow("img", img)
        cv2.imshow('thresh_img',thresh_img)

        cv2.imshow('thresh', thresh)

        cv2.imshow("blank_img", blank_img)

        Lines = cv2.HoughLinesP(blank_img,
                                1.0,
                                np.pi / 180,
                                100,
                                minLineLength=10,
                                maxLineGap=15)
        final_image = draw_lines(blank_img,
                                 Lines,
                                 color=[0, 255, 255],
                                 thickness=4)  #for test
        final_image, Final_line, good = group_lines_and_draw(
        blank_img, Lines, 'Right')
        cv2.imshow("origine line",final_image)
        print(cX, cY)

        cv2.waitKey(0)



if __name__ == '__main__':

    Chest = LoadStreams(2)
    while True:
        img = Chest.imgs
        bridge(img)
        cv2.imshow('?',img)
        cv2.waitKey(0)

    bridge(img)
    cv2.imshow('c',c)
    cv2.waitKey(0)