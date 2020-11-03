import cv2
import math
import numpy as np
import threading
import time
import datetime
import CMDcontrol
from CMDcontrol import CMD_transfer, action_list
from CMDcontrol import action_append

from Center import Back_to_center
from Start_door import start_door
from Undistort import undistort_chest,undistort_head
from State_machine import StateMachine,Event_transitions
from Turn import turn
# from Hole import hole
# from Landmine import landmine
from bafflenew import baffle_function_lza
from landmine_lyu import landmine_lyu
from Bridge import bridge
from Door import door
from Flap import flap,baffle,afterbaffle_turn
# from Step import step
from Holeb import holeb
from Holeg import holeg
from Stage import Stage
from Ball import Ball
from End_door import end_door
from Video_stream import LoadStreams
#################################################初始化
########关卡State函数#########
def start_door_state(Event_list):  #Check
    print('This is start_door_state')
    start_door(Chest)
    return ("Trans",Event_list)

def turn_state(Event_list):  #problem
    print('This is turn_state')
    turn()
    Back_to_center(Chest,'Right')
    return ("Trans",Event_list)

def holeb_state(Event_list):    #check
    print('This is holeb_state')
    # Back_to_center(Chest)
    holeb(Chest)
    return ("Trans",Event_list)

def landmine_state(Event_list):  #Check
    print('This is landmine_state')
    Back_to_center(Chest)
    landmine_lyu(Chest)
    return ("Trans",Event_list)

def bridge_state(Event_list):  #check
    print('This is bridge_state')
    bridge(Chest)
    return ("Trans",Event_list)

def door_state(Event_list):   #Check
    print('This is door_state')
    door(Head)
    return ("Trans",Event_list)

def flap_state(Event_list):  #Check
    print('This is flap_state')
    baffle_function_lza(Chest,True) #转弯？转弯的话不需要专门的turn
    return ("Trans",Event_list)


def ball_state(Event_list):  #Check
    print('This is ball_state')
    Ball()
    return ("Trans", Event_list)


def step_state(Event_list):   #Problem
    print('This is step_state')
    Stage(Chest,Head)
    return ("Trans",Event_list)

def End_door_state(Event_list):  #Check
    print('This is End_door_state')
    end_door(Head)
    return ("Trans",End_door_state)

################################################机器人主函数#################################################

if __name__ == '__main__':

    ###########读取视频流

    Chest = LoadStreams(0)
    Head = LoadStreams(2)

    # 通信请求线程
    th2 = threading.Thread(target=CMD_transfer)
    th2.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
    th2.start()

    while len(CMDcontrol.action_list) > 0 :
        print("等待启动")
        time.sleep(1)
    action_append("HeadTurnMM")


    m = StateMachine()

    m.add_state("Trans", Event_transitions)

    m.add_state("start_door", start_door_state)
    m.add_state("turn", turn_state)
    m.add_state("holeb", holeb_state)
    m.add_state("landmine", landmine_state)
    m.add_state("flap",flap_state)
    m.add_state("bridge", bridge_state)
    m.add_state("step", step_state)
    m.add_state("door", door_state)
    m.add_state("ball", ball_state)
    m.add_state("End_door", End_door_state, end_state=1)  # 添加最终状态

    m.set_start("start_door") # 设置开始状态

        #     1: "holeg",
        # 2: "landmine",
        # 3: "flap",
        # 4: "turn",
        # 5: "door",
        # 6: "bridge",
        # 7: "ball",
        # 8: "step",
        # 9: "holeb"

    event_order = [9,2,3,5,6,7,8]
    # event_order = [6,2,3,5,1,7,8]
    m.run(event_order)
