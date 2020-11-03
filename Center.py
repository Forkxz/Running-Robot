import cv2
import numpy as np
from math import *
# from CMDcontrol import action_append,action_list
from Robot_control import action_append,action_list
from Undistort import undistort_chest
from Video_stream import LoadStreams
import time
def Calculate_position(line):
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
    R1 = np.asarray([[cos(ang1), sin(ang1), 0], [-sin(ang1), cos(ang1), 0], [0, 0, 1]], dtype=float)
    R2 = np.asarray([[cos(ang2), 0, -sin(ang2)], [0, 1, 0], [sin(ang2), 0, cos(ang2)]], dtype=float)
    R3 = np.asarray([[cos(ang3), sin(ang3), 0], [-sin(ang3), cos(ang3), 0], [0, 0, 1]], dtype=float)

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
    aXZ = np.sign(-Xc[0][0]*Xc[1][0]) * acos(cosXZ) # aXZ+, XZplane youdi zuogao in photo
    tanXZ = tan(aXZ)

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
    ayd = np.sign(u-ud) * acos(cosyd) # ud < u, ayd+, car rotates to the right
    ayddeg = ayd/pi*180

    # rotation matrix of y-rotation
    Ry = [[cos(ayd), 0, -sin(ayd)], [0, 1, 0], [sin(ayd), 0, cos(ayd)]]

    # calculate x-translation of the car #######################################
    # rotate Zc to let photo dibian parallel to XZplane, angle = aXZ
    tl = y_fen / (ub-ut) # before rotation
    al = atan(tl)
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
    ak = np.power(np.linalg.norm(vecbot0, ord=2), 4)*(np.power(cos(alz),2)-1)
    bk = 2*np.power(np.linalg.norm(vecbot0, ord=2), 2)*np.matmul(vecbot0,vecmid0)*(1-np.power(cos(alz), 2))
    ck = np.power(np.linalg.norm(vecmid0, ord=2)*np.linalg.norm(vecbot0, ord=2)*cos(alz), 2) - np.power(np.matmul(vecbot0,vecmid0), 2)

    bk, ck = bk[0][0], ck[0][0]

    k1 = (-bk+sqrt(bk*bk - 4*ak*ck))/(2*ak)
    k2 = (-bk-sqrt(bk*bk - 4*ak*ck))/(2*ak)

    xd = max(k1,k2)

    return xd,ayddeg

def draw_lines(img,lines,color=[255,0,0],thickness=3):
        """
        划线
        """
        if lines is None:
            return

        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(img,(x1,y1),(x2,y2),color,thickness)
        return img
        
def group_lines_and_draw(img,lines,side):
        """
        根据斜率，将所有的线分为左右两组,斜率绝对值小于0.5的舍去（减少噪声影响）
        （因为图像的原点在左上角，slope<0是left lines，slope>0是right lines)
        设定min_y作为left和right的top线,max_y为bottom线，求出四个x值即可确定直线：
        将left和right的点分别线性拟合，拟合方程根据y值，求出x值，画出lane
        """
        out_line = None
        line_x,line_y=[],[]
        for line in lines:
            for x1,y1,x2,y2 in line:
                slope=(y2-y1)/(x2-x1)

                if side == 'Right':
                    if abs(slope)>0.4 and slope>0:
                        line_image=draw_lines(img,[[[x1,y1,x2,y2]]],color=[0,0,255],thickness=3)
                        line_x.extend([x1,x2])
                        line_y.extend([y1, y2])
                        # line_slope.extend([slope])
                elif side == 'Left':
                    if abs(slope)>0.4 and slope<0:  
                        line_image=draw_lines(img,[[[x1,y1,x2,y2]]],color=[0,0,255],thickness=3)
                        line_x.extend([x1,x2])
                        line_y.extend([y1, y2])
                        # line_slope.extend([slope])
       
        #设定top 和 bottom的y值,y值都一样,根据ROI变化
        min_y=int(-250)
        max_y=int(img.shape[0]+90)

        #对所有点进行线性拟合
        if len(line_x)==0:
            good = False
            line_image = None
        else:
            poly_left = np.poly1d(np.polyfit(line_y, line_x, deg=1))
            # Fit_slope = poly_left.coef[0]
            # line_slope.extend([Fit_slope])
            # var = np.var(line_slope)
            # print('var',var)
            x_start = int(poly_left(max_y))
            x_end = int(poly_left(min_y))
            out_line = [x_start,x_end]
            good = True
            line_image=draw_lines(img,[[
                [x_start,max_y,x_end,min_y],          
                ]],thickness=2)

        return line_image,out_line,good

def Move_dicision(dx,Deg,Side='Right'):
    if Side=='Right':
        Distance_off = dx - 30
        Deg_off = Deg

        Step = abs(Distance_off) // 2
        Trun = abs(Deg_off) // 5

        if Distance_off > 0:
            Move_action = 'Right02move'
        else:
            Move_action = 'Left02move'

        if Deg_off < 0:
            Turn_action = 'turn001R'   
        else:
            Turn_action = 'turn001L'
        # print('怎么走？' , Step, Trun, Move_action, Turn_action)    
    elif Side=='Left':
        Distance_off = 30 - dx 
        Deg_off = Deg

        Step = abs(Distance_off) // 2
        Trun = abs(Deg_off) // 5

        if Distance_off < 0:
            Move_action = 'Left02move'
        else:
            Move_action = 'Right02move'

        if Deg_off < 0:
            Turn_action = 'turn001R'   
        else:
            Turn_action = 'turn001L'
        # print('怎么走？' , Step, Trun, Move_action, Turn_action) 

    return Step,Trun,Move_action,Turn_action

def Back_to_center (Chest_img,wich_side='Left'):
    """
    split left and right to calculate
    return list = Step,Trun,YouJinShen,Right,Clockwise

    """
    Filter_length = 130
    iteration = 0
    while True:
        if len(action_list) == 0:
            print('Filter_length',Filter_length)
            Chest = np.rot90(undistort_chest(Chest_img.imgs)).copy()
            cv2.imshow("undistort_chest", Chest)
            cv2.waitKey(1)
            # continue
            if wich_side == 'Right':
                ROI_image = Chest[250:550,240:450]#右侧边缘，胸部
            elif wich_side == 'Left':
                ROI_image = Chest[250:550,30:239]#左侧边缘，胸部

            # 机器人脚的位置
            # ROI_image[340,:] = 255 

            cv2.imshow("Chest_img",ROI_image)
            cv2.waitKey(1)

            ROI_image = cv2.pyrMeanShiftFiltering(ROI_image, 9, 25)
            cv2.imshow("pyrMeanShiftFiltering",ROI_image)
            cv2.waitKey(1)
            Canny_img = cv2.Canny(ROI_image,15,150)
            # cv2.imshow("Canny_img",Canny_img)
            # cv2.waitKey(1)

            #膨胀加粗边缘 
            dilate = cv2.dilate(Canny_img, np.ones((2, 2), np.uint8), iterations=1)
            cv2.imshow("dilate",dilate)
            cv2.waitKey(1)


            Lines = cv2.HoughLinesP(dilate,1.0,np.pi / 180, 100,minLineLength=Filter_length,maxLineGap=15)

            # final_image = draw_lines(ROI_image,Lines,color=[0,255,0],thickness=2) #for test
            # cv2.imshow("origine line",final_image)
            # cv2.waitKey(1)
            final_image, Final_line, good = group_lines_and_draw(ROI_image, Lines, wich_side)
            if Final_line is None:
                if Filter_length > 80:
                    Filter_length -= 10
                else:
                    iteration += 1
                continue
            
            if iteration == 3:
                print('No lines for long, just go')
                break

            cv2.imshow("image line",final_image)
            cv2.waitKey(1)
            # print('test')
            if good:
                if wich_side == 'Right':
                    Final_line[0] = Final_line[0] + 240
                    Final_line[1] = Final_line[1] + 240
                if wich_side == 'Left':
                    Final_line[0] = Final_line[0] + 30
                    Final_line[1] = Final_line[1] + 30
                dX, deg = Calculate_position(Final_line)
                # print('line info',dX,deg)
                Step, Trun, Move_action, Turn_action = Move_dicision(dX, deg, wich_side)
                if Step == 0 and Trun == 0:
                    print('In the center')
                    break        
            else:
                Step,Trun,Move_action,Turn_action = 0,0,True,True
                print('啥也没看见朋友！')
            

            for i in range(int(Trun)):
                action_append(Turn_action)
                time.sleep(0.5)

            for i in range(int(Step)):
                action_append(Move_action)
                time.sleep(0.5)

def test_video(dataset):
    while True:
        cv2.imshow('img0', dataset.imgs)
        if cv2.waitKey(1) == ord('q'):  # q to quit
            raise StopIteration
if __name__ == '__main__':
    
    # Chest_path = '../code/track_picture/test/1005chest5.png'
    # Chest_img = cv2.imread(Chest_path,1)
    # Chest_img = cv2.resize(Chest_img, (640, 480))
    Chest = LoadStreams(1)
    # cv2.imshow("Chest_img",Chest_img)
    # cv2.waitKey(1)
    Back_to_center(Chest,'Left')
