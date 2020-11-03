# import CMDcontrol

# ################################################动作执行线程
# def move_action():
#     CMDcontrol.CMD_transfer()
action_list = []
acted_name = ""
def action_append(act_name):
    global acted_name
    print('exe action: ',act_name)
    # if act_name == "forwardSlow0403" and (acted_name == "Forwalk02RL" or acted_name == "Forwalk02L"):
    #     acted_name = "Forwalk02LR"
    # elif act_name == "forwardSlow0403" and (acted_name == "Forwalk02LR" or acted_name == "Forwalk02R"):
    #     acted_name = "Forwalk02RL"
    # elif act_name != "forwardSlow0403" and (acted_name == "Forwalk02LR" or acted_name == "Forwalk02R"):
    #     # CMDcontrol.action_list.append("Forwalk02RS")
    #     # acted_name = act_name
    #     print(act_name,"动作未执行 执行 Stand")
    #     acted_name = "Forwalk02RS"
    # elif act_name != "forwardSlow0403" and (acted_name == "Forwalk02RL" or acted_name == "Forwalk02L"):
    #     # CMDcontrol.action_list.append("Forwalk02LS")
    #     # acted_name = act_name
    #     print(act_name,"动作未执行 执行 Stand")
    #     acted_name = "Forwalk02LS"
    # elif act_name == "forwardSlow0403":
    #     acted_name = "Forwalk02R"
    # else:
    #     acted_name = act_name
    # CMDcontrol.actionComplete = False
    # if len(CMDcontrol.action_list) > 0 :
    #     print("队列超过一个动作")
    #     CMDcontrol.action_list.append(acted_name)
    # else:
    #     CMDcontrol.action_list.append(acted_name)
    # CMDcontrol.action_wait()
