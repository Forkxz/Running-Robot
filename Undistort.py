import cv2
import numpy as np

DIM=(640, 480)
K_head=np.array([[192.04460179257674, 0.0, 320.10071266489734], [0.0, 190.9455211809197, 230.13801374846804], [0.0, 0.0, 1.0]])
D_head=np.array([[0.5897757067426853], [0.3073866728329003], [-0.6844739766900497], [0.3477564084716027]])
K_chest=np.array([[200.7757953492205, 0.0, 323.1118385876546], [0.0, 199.78115382623156, 250.25519357372963], [0.0, 0.0, 1.0]])
D_chest=np.array([[0.7033582961523535], [-0.10502550442184176], [-0.06888804261522179], [0.025679879517408646]])

def undistort_head(img):
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K_head, D_head, np.eye(3), K_head, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)    
    return undistorted_img

def undistort_chest(img):
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K_chest, D_chest, np.eye(3), K_chest, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)    
    return undistorted_img
