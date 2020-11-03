import cv2
from Center import Back_to_center

class StateMachine:
    def __init__(self): 
        self.handlers = {}        # 状态转移函数字典
        self.startState = None    # 初始状态
        self.endStates = []       # 最终状态集合
    
    # 参数name为状态名,handler为状态转移函数,end_state表明是否为最终状态
    def add_state(self, name, handler, end_state=0):
        
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = name

    def run(self, cargo):
        try:
            handler = self.handlers[self.startState]
        except:
            raise Exception("must call .set_start() before .run()")
        if not self.endStates:
            raise  Exception("at least one state must be an end_state")
        
        # 从Start状态开始进行处理
        while True: 
            (newState, cargo) = handler(cargo)     # 经过状态转移函数变换到新状态
            if newState in self.endStates: # 如果跳到终止状态,则打印状态并结束循环
                handler = self.handlers[newState]
                print("reached ", newState)
                break 
            else:                        # 否则将转移函数切换为新状态下的转移函数 
                handler = self.handlers[newState]
event_dist = {
        1: "holeg",
        2: "landmine",
        3: "flap",
        4: "turn",
        5: "door",
        6: "bridge",
        7: "ball",
        8: "step",
        9: "holeb"}

# 自定义状态转变函数
def Event_transitions(Event_list):
    if len(Event_list) == 0:
        newState = 'End_door'
    else:
        newState = event_dist[Event_list[0]] 
        del Event_list[0]
    return (newState, Event_list)        # 返回新状态和余下的关卡


def start_door_state(Event_list):
    print('This is start_door_state')
    return ("Trans",Event_list)

def hole_state(Event_list):
    print('This is hole_state')
    Back_to_center(Input_img,'Right')
    return ("Trans",Event_list)    

def landmine_state(Event_list):
    print('This is landmine_state')
    return ("Trans",Event_list)  

def End_door_state(Event_list):
    print('This is End_door_state')
    return ("Trans",End_door_state) 

if __name__== "__main__":
    m = StateMachine()

    m.add_state("Trans", Event_transitions) 

    m.add_state("start_door", start_door_state) 

    m.add_state("hole", hole_state)
    m.add_state("landmine", landmine_state)

    m.add_state("End_door", End_door_state, end_state=1)  # 添加最终状态
    
    m.set_start("start_door") # 设置开始状态

    event_order = [2, 1]

    Chest_path = '../track_picture/test/1005chest5.png'
    Input_img = cv2.imread(Chest_path,1)
    Input_img = cv2.resize(Input_img,(640,480))
    cv2.imshow("Chest_img",Input_img)
    cv2.waitKey(1)
    m.run(event_order)