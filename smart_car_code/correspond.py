import serial
import numpy as np
import time

# com = serial.Serial(port="COM6", baudrate=115200,
#                     bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)

def getdigit(v):
    v_temp = v
    if v < 0:v = -v
    digit = np.array([])
    digit = np.append(digit, str(int(v % 10)))
    v = int(v/10)
    digit = np.append(digit, str(int(v % 10)))
    v = int(v/10)
    digit = np.append(digit, str(int(v % 10)))
    if v_temp >= 0: digit = np.append(digit, '0')
    else: digit = np.append(digit, '-')
    return digit


def M_send(v_x, v_y, angular):
    data='M'
    digit = getdigit(v_x)
    for i in range(3, -1, -1):
        data+=str(digit[i])
    digit = getdigit(v_y)
    for i in range(3, -1, -1):
        data+=str(digit[i])
    digit = getdigit(angular)
    for i in range(3, -1, -1):
        data+=str(digit[i])
    data+='~'
    print(data)
    com.write(data.encode('ascii'))#使用write函数将命令写入串口


def servo(digit1, digit2, digit3):#1顺时针；2逆时针
    data='S'
    digit = getdigit(digit1)
    for i in range(3, -1, -1):
        data+=str(digit[i])
    digit = getdigit(digit2)
    for i in range(3, -1, -1):
        data+=str(digit[i])
    digit = getdigit(digit3)
    for i in range(3, -1, -1):
        data+=str(digit[i])
    data+='~'
    print(data)
    # data = 'S000000000000~'
    com.write(data.encode('ascii'))#使用write函数将命令写入串口
    time.sleep(0.91)#0.91 180°；0.5 90°
    data = 'S000000000000~'
    com.write(data.encode('ascii'))#使用write函数将命令写入串口


def servo(flag, angle):#1顺时针；0逆时针
    if flag == 0:
        data = 'S160000000000~'
    else:
        data = 'S109000000000~'
    com.write(data.encode('ascii'))#使用write函数将命令写入串口
    if angle == 90:
        time.sleep(0.5)#0.91 180°；0.5 90°
    elif angle == 180:
        time.sleep(0.91)  # 0.91 180°；0.5 90°
    data = 'S000000000000~'
    com.write(data.encode('ascii'))#使用write函数将命令写入串口

def test():
    M_send(0, 80, 0)
    time.sleep(3)
    M_send(-80, 0, 0)
    time.sleep(3)
    M_send(0, 0, 0)

# test()
# M_send(0, 64, 0)
# time.sleep(5)
# M_send(0, 0, -30)
# time.sleep(5)
# M_send(0, 0, 0)
# S_send(160, 0, 0)#36-130-180顺到逆#160逆时针90°109逆时针90°
# time.sleep(1)
# S_send(-100, 0, 0)
# time.sleep(1)
# S_send(0, 0, 0)
    # recive_sta = com.readall()  # 从串口中读取数据
    # print("receive:", recive_sta)
    # Measure_tem_comand = '55 AA 11 00 03 13'  # 要发送的十六进制字符串，两个数字（字母）一组，用空格隔开
    # byte_Measure_tem_comand = bytes.fromhex(Measure_tem_comand)  # 使用fromhex函数将16进制指令转换为byte类型
    # com.write(byte_Measure_tem_comand)  # 使用write函数将命令写入串口
# recive_sta = com.readall()  # 从串口中读取数据

# if recive_sta != b'':  # 判断是否有读取到数据。
        # 将接受的16进制数据格式如b'h\x12\x90xV5\x12h\x91\n4737E\xc3\xab\x89hE\xe0\x16'
        #                      转换成b'6812907856351268910a3437333745c3ab896845e016'
        #                      通过[]去除前后的b'',得到我们真正想要的数据
    # print("receive", str(binascii.b2a_hex(recive_sta))[2:-1])
# M 0000 0000 0000 ~
# S
# E ~