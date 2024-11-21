#!/usr/bin/python3
# -*- coding: utf-8 -*-
import re
import sys
import time
from config_new import *
# 导入路径
import set_path
from cart.widgets import *
from cart.chassis import *
from camera import Camera
from detector_1 import *
# from task_func import *

# 待确定，三个舵机start_init config里舵机对应打角值
# 宏定义
STATE_IDLE = "idle"
STATE_CRUISE = "cruise"
STATE_LOCATE_TASK = "sign_detected"
STATE_DO_TASK = "task"
normal_speed = 20
slow_speed = 10
sign_list = [0] * 10
cam_dir = 1  # 左边为0，右边为1
FRONT_CAM = 2  # 前摄像头编号
SIDE_CAM1 = 0  # 左边摄像头编号
SIDE_CAM2 = 1  # 右边摄像头编号

# 外设初始化
front_camera = Camera("/dev/CAM2", [640, 480])  # 前视摄像头 0
side_camera_left = Camera("/dev/CAM0", [640, 480])  # 侧视摄像头 1
side_camera_right = Camera("/dev/CAM1", [640, 480])  # 侧视摄像头 1
side_camera = [side_camera_left,side_camera_right]
cart = Chassis(1)  # 四个轮子ID1           1   2
#	   		    				          3   4
otherchassis = Chassis(2)  # ID2  升降电机 1 伸缩电机 2
flag_servo = Servo(2)  # 举旗舵机
camera_servo = Servo_pwm(2)  # 摄像头舵机
hit_servo = Servo_pwm(3)  # 击打舵机
grab_servo = Servo_pwm(1)  # 抓取舵机
light = Light(4)  # RGB
echo = UltrasonicSensor(2)  # 超声波
mg = Magneto_sensor(3)  # 光电
buzzer = Buzzer()  # 蜂鸣器    buzzer.rings()叫一下
start_button = Button_angel(1, "2")  # 开始按键
stop_button = Button_angel(1, "4")  # 结束按键

# 模型初始化
cruiser = Cruiser('./detector/model/cruise')  # 巡航模型
cruiser_fsyl = Cruiser('./detector/model/cruise_fsyl')
sign_detector = SignDetector()  # 地面模型
task_detector = TaskDetector()  # 侧面模型
hit_detector = HitDetector()  # 击打模型
purchase_detector = PurchaseDetector()  # 采购模型
trade_detector = TradeDetector()  # 以物易物模型
castle_detector = CastleDetector()  # 以物易物模型


def pairing_task(index):
    for i in range(1, 6):
        if task[i]['sign'] == index:
            return i
    return 0


def check_stop():
    if stop_button.clicked():
        return True
    return False


# def decide_fsyl():
#     front_img = front_camera.read()
#     res = sign_detector.detect(front_img)
#     for sign in res:
#         if (sign.box[2] - sign.box[0])*(sign.box[3] - sign.box[1]) < 90000:
#             return 0


def lift(dir, lift_speed):
    Lift_Speed = [int(dir * lift_speed), 0, 0, 0]
    otherchassis.move(Lift_Speed)


def lift_height(targetheight, speed):     # 升降调节到固定高度
    while True:
        currentheight = echo.read()
        print("------------------>>distance=", currentheight)
        sys.stdout.flush()
        if targetheight > currentheight:
            dir = 1
        elif targetheight < currentheight:
            dir = -1
        elif targetheight == currentheight:
            otherchassis.stop()
            return 1
        lift(dir, speed)


def grab(speed):  # 抓取物体并收回
    T1 = time.time()
    T2 = T1
    while(T2-T1 < 3):
        data = mg.read()
        print(data)
        sys.stdout.flush()
        if data > 15:
            Speed = [0, speed, 0, 0]        # + 伸出
            otherchassis.move(Speed)
        elif data <= 15:
            otherchassis.stop()
            return 1
        T2 = time.time()
    return 1


def height_adjustment(speed, cam_num ,cur_tsk=3):  # 高度对齐小人
    # 高度映射
    # 左平移
    if cur_tsk == 3:
        center_y = 350
        while True:
            image = side_camera[cam_num].read()
            res = hit_detector.detect(image)
            if len(res) != 0:
                image, center, index = detection_img(res, image)
                if center != None:
                    print(center)
                    sys.stdout.flush()
                    if center[0][1] > center_y + 5:
                        lift(-1, speed)
                    elif center[0][1] < center_y - 5:
                        lift(1, speed)
                    elif center[0][1] <= center_y + 5 and center[0][1] >= center_y - 5:
                        lift(1, 0)
                        return 1
    # elif cur_tsk == 6:
    #     center_y = 443
    #     while True:
    #         image = side_camera.read()
    #         res = trade_detector.detect(image)
    #         if len(res) != 0:
    #             image, center, index_ = detection_img(res, image)
    #             try:
    #                 target = index_.index(1)
    #             except(ValueError):
    #                 target = index_.index(2)
    #             if center != None:
    #                 print(center)
    #                 sys.stdout.flush()
    #                 if center[target][1] > center_y + 5:
    #                     lift(-1, speed)
    #                 elif center[target][1] < center_y - 5:
    #                     lift(1, speed)
    #                 else:
    #                     lift(1, 0)
    #                     return 1


ball_color = 'yellow'
color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'blue': {'Lower': np.array([80, 30, 40]), 'Upper': np.array([120, 150, 150])},
              'green': {'Lower': np.array([35, 43, 46]), 'Upper': np.array([77, 255, 255])},
              'orange': {'Lower': np.array([11, 43, 46]), 'Upper': np.array([25, 255, 255])},
              'yellow': {'Lower': np.array([20, 43, 46]), 'Upper': np.array([34, 255, 255])},
              }


def find_side_center(side_image, cur_tsk):
    if cur_tsk == 1:
        gs_frame = cv2.GaussianBlur(
            side_image, (5, 5), 0)                     # 高斯模糊
        # 转化成HSV图像
        hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
        erode_hsv = cv2.erode(
            hsv, None, iterations=2)                   # 腐蚀 粗的变细
        inRange_hsv = cv2.inRange(
            erode_hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])  # 将特定颜色以外的颜色全部变黑 特定颜色变白
        cnts = cv2.findContours(
            inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if cnts == None:
            return None
        c = max(cnts, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        left_point_x = np.min(box[:, 0])
        right_point_x = np.max(box[:, 0])
        top_point_y = np.min(box[:, 1])
        bottom_point_y = np.max(box[:, 1])
        area = (right_point_x-left_point_x)*(bottom_point_y-top_point_y)
        if area < 7000:
            return None
        center_x = center_x = (right_point_x+left_point_x)/2
        return center_x

    if cur_tsk == 2:  # 文化交流
        res_side = castle_detector.detect(side_image)
        print(res_side)
        sys.stdout.flush()
        center = None
        if len(res_side) > 0:
            image, center, index = detection_img(res_side, side_image)
            return center[0][0]
        else:
            return None

    if cur_tsk == 3:  # 保护丝路
        res_side = hit_detector.detect(side_image)
        print(res_side)
        sys.stdout.flush()
        center = None
        if len(res_side) > 0:
            image, center, index = detection_img(res_side, side_image)
            if index[0] == 3 or index[0] == 4:
                return -1
            else:
                return center[0][0]
        else:
            return None

    if cur_tsk == 6:  # 以物易物
        res_side = trade_detector.detect(side_image)
        print(res_side)
        sys.stdout.flush()
        center = None
        if len(res_side) > 0:
            image, center, index_ = detection_img(res_side, side_image)
            return center[0][0]
        else:
            return None


def steer_time(target_time,speed):
    T1 = time.time()
    T2 = T1
    while(T2-T1 < target_time):
        front_image = front_camera.read()
        angle = cruiser.infer_cnn(front_image)
        cart.set_speed(speed)
        cart.steer(angle)
        T2 = time.time()


def purchase(cur_tsk):
    lift_height(task[cur_tsk]['height1'], 100)
    grab_servo.servo_control(40, 40)
    cart.translation(15)
    time.sleep(0.5)
    cart.stop()
    if grab(40):
        grab_servo.servo_control(0, 80)
        lift_height(task[cur_tsk]['height2'], 100)
        speed = [0, -30, 0, 0]
        otherchassis.move(speed)
        time.sleep(1)
        otherchassis.stop()
    cart.translation(-20)
    time.sleep(0.5)
    cart.stop()
    lift_height(5, 80)


def cultural_exchange():
    cart.stop()
    location = 0
    side_im = side_camera[0].read()
    res = castle_detector.detect(side_im)
    if len(res) > 0:
        image, center, index_ = detection_img(res, side_im)
        location = index_[0]
    if location == 3:
        flag_servo.servo_control(-50, 80)
    elif location == 1:
        flag_servo.servo_control(120, 80)
    elif location == 2:
        flag_servo.servo_control(-140, 80)
    for i in range(3):
        light.light_control(0, 0, 255, 0)
        time.sleep(0.2)
        light.light_off()
        time.sleep(0.2)


def protect(cur_tsk, cur_hit, c_dir,cam_num):
    flag = height_adjustment(80)
    slide = 0
    if(flag):
        cart.stop()
        side_image = side_camera[cam_num].read()
        res = hit_detector.detect(side_image)
        for sign in res:
            if (sign.box[2] - sign.box[0])*(sign.box[3] - sign.box[1]) > 12000:
                cart.translation(15*c_dir)
                time.sleep(0.6)
                cart.stop()
                slide = -1
            elif(sign.box[2] - sign.box[0])*(sign.box[3] - sign.box[1]) < 7000:
                cart.translation(15*-c_dir)
                time.sleep(0.6)
                cart.stop()
                slide = 1
            else:
                pass
        # if cur_hit != 1:
        #     tlspeed = 15
        #     keeptime = 0.5
        #     cart.stop()
        #     cart.translation(tlspeed*-c_dir)
        #     time.sleep(keeptime)
        #     cart.stop()
        while True:
            print("adjing!")
            sys.stdout.flush()
            side_image = side_camera[cam_num].read()
            finded_center_x = find_side_center(side_image, cur_tsk)
            if finded_center_x != None:
                if finded_center_x < task[cur_tsk]['hit_x'][cur_hit]-15:
                    cart.set_speed(slow_speed*c_dir*(-1))
                    cart.steer(0)
                elif finded_center_x > task[cur_tsk]['hit_x'][cur_hit]+15:
                    cart.set_speed(slow_speed*c_dir)
                    cart.steer(0)
                else:
                    cart.stop()
                    print("ok")
                    sys.stdout.flush()
                    break
        time.sleep(1)
        hit_servo.servo_control(0, 80)
        time.sleep(1)
        hit_servo.servo_control(105, 80)
        time.sleep(1)
        if slide != 0:
            cart.translation(18*c_dir*slide)
            time.sleep(0.5)
            cart.stop()
        lift_height(5, 80)


def Singandfriendship(cur_tsk=4):
    # speed = [20, 20, 20, 20]
    # cart.move(speed)
    # time.sleep(2.5)
    steer_time(2)
    cart.translation(-20)
    time.sleep(2.8)
    cart.stop()

    for i in range(3):
        light.light_control(0, 255, 0, 0)
        buzzer.rings()
        time.sleep(0.5)
        light.light_off()
        time.sleep(0.5)

    cart.translation(20)
    time.sleep(2.8)
    cart.stop()

def Over_the_mountains():
    print("o")
    sys.stdout.flush()
    front_image = front_camera.read()
    res = sign_detector.detect(front_image)
    cart.set_speed(15)        
    img, center, index = detection_img(res,front_image)
    while True:
        front_image = front_camera.read()
        angle = cruiser_fsyl.infer_cnn(front_image)
        res = sign_detector.detect(front_image)
        img, center, index = detection_img(res,front_image)
        cart.set_speed(15)
        cart.steer(angle)
        print("up!")
        print(res)
        sys.stdout.flush()
        if len(res) > 0:
            if center[0][1] >= 300:
                break

    T1 = time.time()
    T2 = T1
    while((T2-T1) < 1):
#        front_image = front_camera.read()
#        angle = cruiser.infer_cnn(front_image)
        cart.set_speed(25)
        cart.steer(0)
        T2 = time.time()
# def Over_the_mountains():
#     print("o")
#     sys.stdout.flush()
#     front_image = front_camera.read()
#     res = sign_detector.detect(front_image)        
#     img, center, index = detection_img(front_image,res)
#     # while index[0] != None:
#     #     front_image = front_camera.read()
#     #     angle = cruiser_fsyl.infer_cnn(front_image)
#     #     res = sign_detector.detect(front_image)        
#     #     img, center, index = detection_img(front_image,res)
#     #     cart.set_speed(15)
#     #     cart.steer(angle)
#     # T1 = time.time()
#     # T2 = T1
#     # while(T2-T1 < 1.5):
#     #     front_image = front_camera.read()
#     #     angle = cruiser.infer_cnn(front_image)
#     #     cart.set_speed(normal_speed)
#     #     cart.steer(angle)
#     #     T2 = time.time()
#     while True:
#         if len(res) !=0:
#             front_image = front_camera.read()
#             angle = cruiser_fsyl.infer_cnn(front_image)
#             res = sign_detector.detect(front_image)
#             img, center, index = detection_img(res,front_image)
#             if center[0][1] <= 100:
#                 break
#             cart.set_speed(15)
#             cart.steer(angle)
#             print("up!")
#             print(res)
#             sys.stdout.flush()
        
# #    steer_time(1,20)
#     T1 = time.time()
#     T2 = T1
#     while((T2-T1) < 2):
# #        front_image = front_camera.read()
# #        angle = cruiser.infer_cnn(front_image)
#         cart.set_speed(20)
#         cart.steer(0)
#         T2 = time.time()

def bartering(cur_tsk,cam_num, c_dir=1):
    cart.stop()
    cart.translation(15)
    time.sleep(0.6)
    cart.stop()
    center_y = 0
    trade_x = 460
    speed = [0, 30, 0, 0]
    otherchassis.move(speed)
    time.sleep(2)
    otherchassis.stop()
    grab_servo.servo_control(40, 40)
    time.sleep(0.5)
    speed = [0, -30, 0, 0]
    otherchassis.move(speed)
    time.sleep(1.5)
    otherchassis.stop()
    grab_servo.servo_control(0, 80)
    while True:
        side_image = side_camera[cam_num].read()
        finded_center_x = find_side_center(side_image, cur_tsk)
        if finded_center_x != None:
            if finded_center_x < trade_x-15:
                cart.set_speed(slow_speed*c_dir)
                cart.steer(0)
            elif finded_center_x > trade_x+15:
                cart.set_speed(slow_speed*c_dir*(-1))
                cart.steer(0)
            else:
                cart.stop()
                print("ok")
                sys.stdout.flush()
                break
    while True:
        side_img = side_camera[cam_num].read()
        res_side = trade_detector.detect(side_img)
        if len(res_side) > 0:
            image, center, index_ = detection_img(res_side, side_img)
            center_y = center[0][1]
            index = index_[0]
            print(center_y)
            sys.stdout.flush()
            break
        else:
            return None
    if center_y > 300 and (index == 1 or index == 2):
        lift_height(6, 80)
    elif center_y < 300 and (index != 1 and index != 2):
        lift_height(6, 80)
    elif center_y > 300 and (index != 1 and index != 2):
        lift_height(14, 80)
    elif center_y < 300 and (index == 1 or index == 2):
        lift_height(14, 80)
#    height_adjustment(40,9)
    grab_servo.servo_control(40, 40)
    cart.stop()
    if grab(40):
        grab_servo.servo_control(0, 80)
        speed = [0, -30, 0, 0]
        otherchassis.move(speed)
        time.sleep(1)
        otherchassis.stop()
    cart.translation(-15)
    time.sleep(0.5)
    cart.stop()
    lift_height(5, 80)
