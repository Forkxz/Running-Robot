import cv2
import numpy as np
from CMDcontrol import action_list,action_append
# from Robot_control import action_append,action_list

def Ball():
    action_append("fastForward04")
    action_append("fastForward04")
    for i in range (8):
        action_append("turn001L")
        while len(action_list) != 0:
            time.sleep(0.1)