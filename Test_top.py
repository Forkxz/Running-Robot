import cv2
import math
import numpy as np
import threading
import time
import datetime
# import CMDcontrol
# from CMDcontrol import CMD_transfer, action_list
# from CMDcontrol import action_append
from Robot_control import action_append,action_list
# from Center import Back_to_center
# from Start_door import start_door
# from Undistort import undistort_chest, undistort_head
# from State_machine import StateMachine, Event_transitions
# from bafflenew import baffle_function_lza
# from Turn import turn
# from Hole import hole
# from Landmine import landmine
# from landmine_lyu import landmine_lyu
from Bridge import bridge
# from Door import door
# from Flap import flap, baffle, afterbaffle_turn
# from Step import step
# from Stage import Stage
# from End_door import end_door
# from bafflenew import baffle_function_lza
from Video_stream import LoadStreams

################################################机器人主函数#################################################

if __name__ == '__main__':

    ###########读取视频流

   #  Chest = LoadStreams('V2/test.mp4')
    Chest = LoadStreams(2)
    # Head = LoadStreams(2)

    # # 通信请求线程
    # th2 = threading.Thread(target=CMD_transfer)
    # th2.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
    # th2.start()

    # while len(CMDcontrol.action_list) > 0 :
    #     print("等待启动")
    #     time.sleep(1)
    # action_append("HeadTurnMM")
    bridge(Chest)
    # hole(Chest)
    # baffle_function_lza(Chest)
    # Stage(Chest,Chest)
