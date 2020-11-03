#!/usr/bin/env python3
# coding:utf-8

import cv2
import serial
import time
import binascii

import threading
import subprocess
import eventlet


PACKAGE_HEAD = [0xFF, 0xFF]
RES_PACKAGE_HEAD = [0xFF, 0xFF]


runningAction = False
stopRunning = False
actionComplete = False      # activate() actionComplete = False

action_list = []

################################################LUA函数请求#############################################

def set_wifi(wifi_name, wifi_password):
    connect_wifi_cmd = "sudo nmcli dev wifi connect {name} password {password}".format(name=wifi_name, password=wifi_password)
    try:
        result = subprocess.check_output(connect_wifi_cmd, shell=True)
        print("result 30L")
        print(result)
        error_bytes = bytes("Error",encoding='utf8')
        if result.startswith(error_bytes):
            res = False
        else:
            res = True
    except Exception as err:
        print("err 37L")
        print(err)
        res = False

    return res


def handle_wifi(param, ser):
    wifi_name = param[0]
    wifi_password = param[1]


    if set_wifi(wifi_name, wifi_password):
        result = [0x00, 0x00]
    else:
        result = [0x00, 0x01]
    # print(result)

    pac_len = [0x00, 0x07]
    pac_param_count = [0x01]
    pac_param = [0x03, 0x02] + result

    pac_data = RES_PACKAGE_HEAD + pac_len + pac_param_count + pac_param
    pac_check_sum = crc_calculate(pac_data)

    res = pac_data + pac_check_sum
    # print(res)
    ser.write(res)



# string to byte
def assemble_string(data):
    byte_param = []
    data_list = list(data)
    funname_data = list(map(lambda x: ord(x), data_list))
    return funname_data


def crc_calculate(package):
    crc = 0
    for hex_data in package:

        b2 = hex_data.to_bytes(1, byteorder='little')
        crc = binascii.crc_hqx(b2, crc)

    return [(crc >> 8), (crc & 255)]    # 校验位两位




def action_request(param, ser):
    global action_list 
    if len(action_list) != 0:
        action_name = action_list.pop(0)
    else:
        action_name = 'default'

    # action_name = 'music3'
    # print('REQUEST!')
    pac_data = assemble_string(action_name)
    total_len = 1 + 2 + len(pac_data) + 2

    pac_param_len = [len(pac_data) + 1]
    pac_param_type = [0x03]
    pac_param = pac_param_len + pac_param_type + pac_data

    pac_len = [( total_len>> 8), (total_len & 255)]
    pac_param_count = [0x01]

    pac_data = RES_PACKAGE_HEAD + pac_len + pac_param_count + pac_param
    pac_check_sum = crc_calculate(pac_data)
    res = pac_data + pac_check_sum
    
    # res_hex = map(lambda x: hex(x), res)
    # print(list(res_hex))
    ser.write(res)


def action_start(param, ser):
    global runningAction
    runningAction = True
    # print('start len(list):',len(action_list))

def action_complete(param, ser):
    global actionComplete
    actionComplete = True
    # print('complete')

def boardreceive_error(param, ser):
    print('boardreceive_error')

def nametest(param, ser):
    print("nametest//////////")
    print(param)




FUNC_MAP = {
    "REQ": action_request,
    "START": action_start,
    "COMPLETE": action_complete,
    "ERROR": boardreceive_error,
    "REtest": nametest,
    "wificonnect": handle_wifi
}
################################################函数#############################################


def action_wait():
    global runningAction, actionComplete
    if actionComplete == False:
        eventlet.monkey_patch()
        with eventlet.Timeout(30,False):
            while len(action_list) != 0 or actionComplete == False:
                time.sleep(0.001)
    runningAction = False


FLOAT_TYPE = 0x01
INT_TYPE = 0x02
STRING_TYPE = 0x03

def parse_float(data, data_length):
    float_length = 4

    if data_length == float_length:
        # single float param
        float_string = ''
        for byte in data:
            float_string = float_string + byte

        return struct.unpack('f', float_string)[0]
    else:
        # float array param
        current_index = 0
        float_array = []

        while current_index < data_length:
            current_data = data[current_index:current_index+float_length]
            current_float = parse_float(current_data, float_length)

            float_array.append(current_float)
            current_index = current_index + float_length

        return float_array

def parse_int(data, data_length):
    int_length = 2

    if data_length == int_length:
        # single int param
        return (data[0] << 8) + data[1]
    else:
        #int array param
        current_index = 0
        int_array = []

        while current_index < data_length:
            current_data = data[current_index:current_index+int_length]
            current_int = parse_int(current_data, int_length)

            int_array.append(current_int)
            current_index = current_index + int_length

        return int_array

def parse_string(data):
    string_param = ''

    for byte in data:
        string_param = string_param + chr(byte)

    return string_param

def parse_cmd(cmd_data, ser):
    #global lock
    #lock = threading.Event()
    # process request name, ignore \0
    cmd_name_len = cmd_data[0]
    cmd_name_data = cmd_data[1:cmd_name_len]
    cmd_name = parse_string(cmd_name_data)
    # print("cmd name is: ", cmd_name)

    # process parameters
    cmd_param = []
    cmd_param_count = cmd_data[cmd_name_len+1]

    current_count = 0
    current_index = cmd_name_len + 2
    while current_count < cmd_param_count:
        current_param_len = cmd_data[current_index]
        current_param_type = cmd_data[current_index+1]
        if current_param_type == FLOAT_TYPE:
            # float param
            current_param_data = recv[current_index+2:current_index+current_param_len+1]
            current_param = parse_float(current_param_data, current_param_len-1)
        elif current_param_type == INT_TYPE:
            # int param
            current_param_data = cmd_data[current_index+2:current_index+current_param_len+1]
            current_param = parse_int(current_param_data, current_param_len-1)
        else:
            # string param
            current_param_data = cmd_data[current_index+2:current_index+current_param_len+1]
            current_param = parse_string(current_param_data)

        cmd_param.append(current_param)
        current_count += 1
        current_index = current_index + current_param_len + 1
        #lock.set()        
    #print("CMD are: ", cmd_name)

    process_func = FUNC_MAP.get(cmd_name)
    process_func(cmd_param, ser)


def CMD_transfer():

    tmp_head = []

    with serial.Serial('/dev/ttyAMA0', 9600) as ser:
        while True:
            cur_byte = ser.read()
            tmp_head.append(ord(cur_byte))
            if len(tmp_head) == 2:
                if tmp_head == PACKAGE_HEAD:
                    # print("receive")
                    length_high = ord(ser.read())
                    length_low = ord(ser.read())
                    package_data_length = (length_high << 8) + length_low
                    package_data_byte = ser.read(package_data_length)
                    package_data = list( package_data_byte)

                    # crc check
                    tmp_package = PACKAGE_HEAD + [length_high, length_low] + package_data[0:len(package_data)-2]
                    package_crc_sum = package_data[len(package_data)-2:]
                    tmp_crc_sum = crc_calculate(tmp_package)
                    # print(tmp_crc_sum)

                    if package_crc_sum == tmp_crc_sum:
                        try:
                            parse_cmd(package_data, ser)
                        except Exception as err:
                            data_hex = map(lambda x: hex(x), package_data)
                            print(list(data_hex))
                            print("CMD_transfer():error")
                    else:
                        print("CMD_transfer():Incorrect checksum!")

                    tmp_head = []
                else:
                    tmp_head.pop(0)


acted_name = ""
def action_append(act_name):
    global acted_name , action_list
    if act_name == "forwardSlow0403" and (acted_name == "Forwalk02RL" or acted_name == "Forwalk02L"):
        acted_name = "Forwalk02LR"
    elif act_name == "forwardSlow0403" and (acted_name == "Forwalk02LR" or acted_name == "Forwalk02R"):
        acted_name = "Forwalk02RL"
    elif act_name != "forwardSlow0403" and (acted_name == "Forwalk02LR" or acted_name == "Forwalk02R"):
        action_list.append("Forwalk02RS")
        acted_name = act_name
    elif act_name != "forwardSlow0403" and (acted_name == "Forwalk02RL" or acted_name == "Forwalk02L"):
        action_list.append("Forwalk02LS")
        acted_name = act_name
    elif act_name == "forwardSlow0403":
        acted_name = "Forwalk02R"
    else:
        acted_name = act_name

    actionComplete = False
    if len(action_list) > 0 :
        print("队列超过一个动作")
        action_list.append(acted_name)
    else:
        action_list.append(acted_name)
    action_wait()
    # print("执行动作名：",act_name)
    # time.sleep(3) # fftest

    

if __name__ == '__main__':
    # 通信请求线程
    th2 = threading.Thread(target=CMD_transfer)
    th2.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
    th2.start()
    
    # CMD_transfer()

    # with serial.Serial('/dev/ttyAMA0', 9600) as ser:
    #     action_request(0,ser)

    act_name = ""
    while True:
        if len(action_list) == 0:
            act_name = input("please act_name:")
            if act_name != None:
                action_append(act_name)
