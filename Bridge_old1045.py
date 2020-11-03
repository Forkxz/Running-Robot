import cv2
import numpy as np
import time
from CMDcontrol import action_append, action_list
# from Robot_control import action_append, action_list
from Landmine import getAreaMaxContour1
from Center import Calculate_position
import math
from Undistort import undistort_chest

def Calculate_position_bridge(line):
    [ub, ut] = line
    if ub == ut:
        ub = ub + 1
    # (ut, ub) = (484,-360)
    # input variables ##########################################################
    # biaoding photo
    (u, v, ang1, ang2, ang3, h) = (243,189,-1.5479,-0.5800,1.5526,20.6104)

    # inside camera ############################################################
    f = 200 # pixel
    x_fen = 480
    y_fen = 640

    # find lanes jiaodian in photo #############################################
    # rotation matrix of camera: R1 rotates first
    R1 = np.asarray([[math.cos(ang1), math.sin(ang1), 0], [-math.sin(ang1), math.cos(ang1), 0], [0, 0, 1]], dtype=float)
    R2 = np.asarray([[math.cos(ang2), 0, -math.sin(ang2)], [0, 1, 0], [math.sin(ang2), 0, math.cos(ang2)]], dtype=float)
    R3 = np.asarray([[math.cos(ang3), math.sin(ang3), 0], [-math.sin(ang3), math.cos(ang3), 0], [0, 0, 1]], dtype=float)

    # Zc[0 0 1]axis vector in no-rotate cam coordinate system: Zc[x y z]
    Zc = np.matmul(np.matmul(np.matmul(R3,R2),R1),np.matrix([[0], [0], [1]]))
    # find projection vector of Zc in Xc0-Zc0 plane
    Zcproj = np.asarray([Zc[0][0], 0, Zc[2][0]], dtype=float)
    # find jiaoxian vector of Xc0-Zc0 plane and photo
    XZplane = np.asarray([-Zc[2][0], 0, Zc[0][0]], dtype=float)

    # Xc[1 0 0]axis vector in no-rotate cam coordinate system: Xc[x y z]
    Xc = np.matmul(np.matmul(np.matmul(R3,R2),R1),np.matrix([[1], [0], [0]]))

    # calculate absolute jiajiao of Xc and XZplane
    Xc.shape = (3,1)
    XZplane.shape = (1,3)
    # print(np.matmul(XZplane, Xc))

    cosXZ = abs( np.matmul(XZplane, Xc)/np.linalg.norm(Xc, ord=2)/np.linalg.norm(XZplane, ord=2) )
    aXZ = np.sign(-Xc[0][0]*Xc[1][0]) * math.acos(cosXZ) # aXZ+, XZplane youdi zuogao in photo
    tanXZ = math.tan(aXZ)

    # calculate jiaodian: (ud, vd)
    # original expression:
    # (vd-v)/(ud-u) = tanXZ
    # (ut-ud)/vd = (ud-ub)/(y_fen-vd)
    ud = ( ut*y_fen + (ut-ub)*(tanXZ*u - v) ) / ( tanXZ*(ut-ub) + y_fen)
    vd = ( tanXZ*y_fen*(ut-u) + y_fen*v ) / ( tanXZ*(ut-ub) + y_fen)

    # calculate y-rotation of the car ##########################################
    # vector of camdevi(u,v)-camorigin
    vecuv = np.asarray([x_fen/2, y_fen/2, f], dtype=float) - np.asarray([u, v, 0], dtype=float)
    vecuvd = np.asarray([x_fen/2, y_fen/2, f], dtype=float) - np.asarray([ud, vd, 0], dtype=float)

    vecuv.shape = (1,3)
    vecuvd.shape = (3,1)

    # calculate absolute jiajiao of two vectors
    cosyd = np.matmul(vecuv,vecuvd)/np.linalg.norm(vecuv, ord=2)/np.linalg.norm(vecuvd, ord=2)
    ayd = np.sign(u-ud) * math.acos(cosyd) # ud < u, ayd+, car rotates to the right
    ayddeg = ayd/np.pi*180

    # rotation matrix of y-rotation
    Ry = [[math.cos(ayd), 0, -math.sin(ayd)], [0, 1, 0], [math.sin(ayd), 0, math.cos(ayd)]]

    # calculate x-translation of the car #######################################
    # rotate Zc to let photo dibian parallel to XZplane, angle = aXZ
    tl = y_fen / (ub-ut) # before rotation
    al = math.atan(tl)
    alz = al-aXZ # after rotation

    # after y-rotation: Zc coordinates changes
    Zcy = Ry*Zc
    # if no x-translation: what is photo bottom line coordinates after rotation
    vecbot0 = np.sign(ut-ub) * np.asarray([1, 0, (-Zcy[0][0]/Zcy[2][0])], dtype=float) # ut > ub: left lane
    # what is photo middle line coordinates
    vecmid0 = np.asarray([0, h, -(h*Zcy[1][0]/Zcy[2][0])], dtype=float)

    vecbot0.shape = (1,3)
    vecmid0.shape = (3,1)

    # solve equation: dot product of vecl0 and vecl (vecl = vecmid0 - k*vecbot)
    ak = np.power(np.linalg.norm(vecbot0, ord=2), 4)*(np.power(math.cos(alz),2)-1)
    bk = 2*np.power(np.linalg.norm(vecbot0, ord=2), 2)*np.matmul(vecbot0,vecmid0)*(1-np.power(math.cos(alz), 2))
    ck = np.power(np.linalg.norm(vecmid0, ord=2)*np.linalg.norm(vecbot0, ord=2)*math.cos(alz), 2) - np.power(np.matmul(vecbot0,vecmid0), 2)

    bk, ck = bk[0][0], ck[0][0]

    k1 = (-bk+math.sqrt(bk*bk - 4*ak*ck))/(2*ak)
    k2 = (-bk-math.sqrt(bk*bk - 4*ak*ck))/(2*ak)

    xd = max(k1,k2)
    
    if ub > ut:
        position = False # right is false
    else:
        position = True # left or middle is true

    return xd,ayddeg,position


def bridge(Chest):
    event_step_bridge = 0
    counter_step = 0
    while True:
        img = undistort_chest(Chest.imgs)
        img = img[50:430,50:400].copy()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        cv2.imshow("img", img)
        cv2.waitKey(1)
        height = img.shape[0]
        width = img.shape[1]
        blank_img = np.zeros((height,width,1), np.uint8)

        h,s,v = cv2.split(img)
        thresh_value,thresh_img = cv2.threshold(s, 90, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(10,10))
        thresh = cv2.morphologyEx(thresh_img,cv2.MORPH_OPEN,kernel)
        #cv2.imshow("thresh", thresh)

        _, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  
        length = len(contours)
        list1 = []
        for i in range(length):
            cnt = contours[i]
            epsilon = 0.02*cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt, 20, True)
            cv2.polylines(blank_img, [approx], 1, (255, 0, 0), 1, cv2.LINE_4)
            if(len(approx) == 4 or len(approx) == 5):
                list1.append(approx)
        area_max_contour, contour_area_max = getAreaMaxContour1(list1)
        M = cv2.moments(area_max_contour)
        cX = int(M["m10"] / (M["m00"] + 0.0001))
        cY = int(M["m01"] / (M["m00"] + 0.0001))  
        midpoint = [cX,cY]
        cv2.polylines(blank_img, [area_max_contour], 1, (255, 0, 255), 1, cv2.LINE_4)
        cv2.circle(blank_img, (cX,cY), 6, (255,255,255), -1)
        #cv2.imshow("blank_img", blank_img)
        print(cX, cY)

        #cv2.waitKey(0)

        if event_step_bridge == 0:
            print("event_step_bridge =", event_step_bridge)

            x_dis = abs(cX - 150)
            print("cX =", cX)
            if cX > 150:
                if x_dis > 50:
                    print("Forwalk02")
                    action_append("Forwalk02")
                    while len(action_list) != 0:
                        time.sleep(0.1)
                elif x_dis > 30:
                    print("Forwalk01")
                    action_append("Forwalk01")
                    while len(action_list) != 0:
                        time.sleep(0.1)
                elif x_dis > 20:
                    print("Forwalk00")
                    action_append("Forwalk00")
                    while len(action_list) != 0:
                        time.sleep(0.1)
    #         else:
    #             print("Back1Run")
    #             action_append("Back1Run")
                
            y_dis = abs(cY - 190)
            print("cY =", cY)
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

        if event_step_bridge == 1:
            print("event_step_bridge =", event_step_bridge)
            lines = cv2.HoughLines(blank_img,1,3.14159/180,30,None,0,0)
            cv2.polylines(blank_img, [area_max_contour], 1, (255, 0, 255), 1, cv2.LINE_4)
            k=np.zeros(2, dtype=np.float32)
            bb=np.zeros(2, dtype=np.float32)
            #cnt=-1
            large=10
            small=10
            if lines is not None:
                for i in range(0, len(lines)):
                    rho = lines[i][0][0]
                    theta = lines[i][0][1]
                    if(theta>np.pi/2):
                        if((theta-np.pi/2)<large):
                            large=theta-np.pi/2
                            t_l=np.pi/2+large
                            a = math.cos(t_l)
                            b = math.sin(t_l)
                            x0 = a * rho
                            y0 = b * rho
                            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                            k[0]=(pt2[1]-pt1[1])/(pt2[0]-pt1[0])
                            bb[0]=float((pt1[1]*pt2[0]-pt2[1]*pt1[0])/(pt2[0]-pt1[0]))                
                    else:
                            if((np.pi/2-theta)<small):
                                small=np.pi/2-theta
                                t_s=np.pi/2-small
                                a = math.cos(t_s)
                                b = math.sin(t_s)
                                x0 = a * rho
                                y0 = b * rho
                                pt3 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                                pt4 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                                k[1]=(pt4[1]-pt3[1])/(pt4[0]-pt3[0]+0.0001)
                                bb[1]=float((pt3[1]*pt4[0]-pt4[1]*pt3[0])/(pt4[0]-pt3[0]+0.0001))

            cv2.line(img,pt1,pt2,(0,255,0),1)
            cv2.line(img,pt3,pt4,(0,255,0),1)
            cv2.imshow("frame", img)
            #cv2.waitKey(0)

            left_x_start = k[0]*590+bb[0]+50
            left_x_end = k[0]*50+bb[0]+50
            left_line = [left_x_end,left_x_start]

            right_x_start = k[1]*590+bb[1]+50
            right_x_end = k[1]*50+bb[1]+50
            right_line = [right_x_end,right_x_start]

            dx_l, deg_l, posi_left = Calculate_position_bridge(left_line)
            print('left',dx_l,deg_l)
            dx_r, deg_r, posi_right = Calculate_position_bridge(right_line)
            print('right',dx_r,deg_r)

            if posi_left == False and posi_right == False:
                print("Right3move")
                action_append("Right3move")
                while len(action_list) != 0:
                    time.sleep(0.1)
            elif posi_left == True and posi_right == True:
                print("Left3move")
                action_append("Left3move")
                while len(action_list) != 0:
                    time.sleep(0.1)
            else:
                if dx_l <= 8 or dx_r >= 12:
                    print("Right02move")
                    action_append("Right02move")
                    while len(action_list) != 0:
                        time.sleep(0.1)
                elif dx_r <= 8 or dx_l >= 12:
                    print("Left02move")
                    action_append("Left02move")
                    while len(action_list) != 0:
                        time.sleep(0.1)

            deg_avg = (deg_l+deg_r)/2
            if deg_avg > 5:
                print("turn000L")
                action_append("turn000L")
                while len(action_list) != 0:
                    time.sleep(0.1)
            elif deg_avg < -5:
                print("turn000R")
                action_append("turn000R")
                while len(action_list) != 0:
                    time.sleep(0.1)
            else:
                event_step_bridge = 2

        if event_step_bridge == 2:
            print("event_step_bridge =", event_step_bridge)
            lines = cv2.HoughLines(blank_img,1,3.14159/180,30,None,0,0)
            cv2.polylines(blank_img, [area_max_contour], 1, (255, 0, 255), 1, cv2.LINE_4)
            k=np.zeros(2, dtype=np.float32)
            bb=np.zeros(2, dtype=np.float32)
            #cnt=-1
            large=10
            small=10
            if lines is not None:
                for i in range(0, len(lines)):
                    rho = lines[i][0][0]
                    theta = lines[i][0][1]
                    if(theta>np.pi/2):
                        if((theta-np.pi/2)<large):
                            large=theta-np.pi/2
                            t_l=np.pi/2+large
                            a = math.cos(t_l)
                            b = math.sin(t_l)
                            x0 = a * rho
                            y0 = b * rho
                            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                            k[0]=(pt2[1]-pt1[1])/(pt2[0]-pt1[0])
                            bb[0]=float((pt1[1]*pt2[0]-pt2[1]*pt1[0])/(pt2[0]-pt1[0]))                
                    else:
                            if((np.pi/2-theta)<small):
                                small=np.pi/2-theta
                                t_s=np.pi/2-small
                                a = math.cos(t_s)
                                b = math.sin(t_s)
                                x0 = a * rho
                                y0 = b * rho
                                pt3 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                                pt4 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                                k[1]=(pt4[1]-pt3[1])/(pt4[0]-pt3[0]+0.0001)
                                bb[1]=float((pt3[1]*pt4[0]-pt4[1]*pt3[0])/(pt4[0]-pt3[0]+0.0001))

            cv2.line(img,pt1,pt2,(0,255,0),1)
            cv2.line(img,pt3,pt4,(0,255,0),1)
            cv2.imshow("frame", img)
    #         cv2.waitKey(0)

            left_x_start = k[0]*640+bb[0]+50
            left_x_end = bb[0]+50
            left_line = [left_x_end,left_x_start]

            right_x_start = k[1]*640+bb[1]+50
            right_x_end = bb[1]+50
            right_line = [right_x_end,right_x_start]

            dx_l, deg_l, posi_left = Calculate_position_bridge(left_line)
            print('left',dx_l,deg_l)
            dx_r, deg_r, posi_right = Calculate_position_bridge(right_line)
            print('right',dx_r,deg_r)

            if dx_l <= 8 or dx_r >= 12:
                print("Right02move")
                action_append("Right02move")
                while len(action_list) != 0:
                    time.sleep(0.1)
            elif dx_r <= 8 or dx_l >= 12:
                print("Left02move")
                action_append("Left02move")
                while len(action_list) != 0:
                    time.sleep(0.1)

            deg_avg = (deg_l+deg_r)/2
            if deg_avg > 5:
                for i in range (int(deg_avg/5) + 1):
                    print("turn000L")
                    action_append("turn000L")
                    while len(action_list) != 0:
                        time.sleep(0.1)
            elif deg_avg < -5:
                for i in range (int(abs(deg_avg)/5) + 1):
                    print("turn000R")
                    action_append("turn000R")
                    while len(action_list) != 0:
                        time.sleep(0.1)
            
            if ({dx_l >= 8 and dx_l <= 10}) and ({dx_r >= 8 and dx_r <= 10}):
                print("Forwalk02")
                action_append("Forwalk02")
                while len(action_list) != 0:
                    time.sleep(0.1)
                counter_step = counter_step + 1
                if cX < 100:
                    action_append("fastForward04")
                    while len(action_list) != 0:
                        time.sleep(0.1)
                    event_step_bridge = 3
        
        if event_step_bridge == 3:
            print('bridge finish')
            break


if __name__ == '__main__':
    
    Chest_path = '../track_picture/test/1005chest5.png'
    Chest_img = cv2.imread(Chest_path,1)
    Chest_img = cv2.resize(Chest_img,(640,480))
    bridge(Chest_img)
    # cv2.imshow("Chest_img",Chest_img)
    # cv2.waitKey(0)