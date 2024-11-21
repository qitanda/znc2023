#!/usr/bin/python3
# -*- coding: utf-8 -*-
from work.global_var import *
import time
from work.config_new import *
# 导入路径
import work.set_path
from work.detector_1 import *
from work.all_func_new import *


def start_init():
    front_camera.start()
    side_camera.start()
    lift_height(4, 80)  # 升降放到初始位置
    grab_servo.servo_control(0, 50)  # 爪子放到初始位置    0闭    40开
    flag_servo.servo_control(40, 80)  # 旗子放到初始位置
    camera_servo.servo_control(0, 80)  # 摄像头转到初始位置   175 左  0 右
    hit_servo.servo_control(100, 80)  # 伸缩结构回到初始位置  0伸开  100收缩  伸出去速度快，收缩速度慢
    light.light_off()  # light.light_control(0,r,g,b)
    time.sleep(0.5)


def check_stop():  # 检查是否按下停止按键
    if stop_button.clicked():
        cart.stop()
        front_camera.stop()
        side_camera.stop()
        return True
    return False


def idle_handler():
    print("idle")
    sys.stdout.flush()
    while True:
        if stop_button.clicked():
            sys.exit(0)
        if start_button.clicked():
            return STATE_CRUISE


def cruise_handler():
    global current_tsk
    global cam_dir
    global current_count
    global current_count_once
    sign_list = [0] * 20
    while True:
        if check_stop():
            with open(file_path, mode='w', encoding='utf-8') as file_obj:
                for i in range(len(current_count)):
                    file_obj.write(str(current_count[i]))
            sys.exit(0)
        front_image = front_camera.read()
        angle = cruiser.infer_cnn(front_image)
        cart.set_speed(normal_speed)
        cart.steer(angle)
        # 侦测车道上有无标志图标
        res = sign_detector.detect(front_image)
        if len(res) != 0:
            print(res)
            sys.stdout.flush()
            for sign in res:
                print(sign.index)
                if pairing_task(sign.index):
                    
                    # 获取标志识别结果，获得所在列表的索引值
                    sign_list[sign.index] += 1
                    print(sign_list[sign.index])
                    # 连续加测到一定次数，认为检测到，进入到任务定位程序
                    if sign_list[sign.index] > REC_NUM:
                        current_tsk = pairing_task(sign.index)
                        print(current_count)
                        sys.stdout.flush()
                        print(current_count_once)
#                        if current_count[current_tsk-1]>len(task[current_tsk]['angle']) or current_count[current_tsk-1] == len(task[current_tsk]['angle']):
                        if current_count[current_tsk-1]>current_count_once[current_tsk-1]:
                            current_count_once[current_tsk-1]+=1
                            return STATE_CRUISE
                        else:    
                            if task[current_tsk]['angle'][current_count[current_tsk-1]] == 175:  # ?待定:     摄像头方向 右边-1 左边1   175左 0右
                                cam_dir = 1
                            else:
                                cam_dir = -1
                        camera_servo.servo_control(task[current_tsk]['angle'][current_count[current_tsk-1]],80)
                        lift_height(4, 80)
                        return STATE_LOCATE_TASK


def locate_task_handler():
    global current_tsk
    global cam_dir
    global current_count
    global current_count_once
    flag = 0
    if current_tsk ==  1 or current_tsk == 2 :
        cart.stop()
        cart.translation(cam_dir*10)
        time.sleep(0.3)
    if current_tsk == 4 or current_tsk == 5:  # 放歌友谊或翻山越岭
        if current_tsk == 4 :
            while True:
                front_img = front_camera.read()
                res = sign_detector.detect(front_img)  
                angle = cruiser.infer_cnn(front_img)
                cart.set_speed(slow_speed)
                cart.steer(angle) 
                img,center,index = detection_img(res,front_img)
                if center[0][1] >= 320:
                    break
#        if current_tsk == 5:
#            if(decide_fsyl() == 0):
#                return STATE_CRUISE
        return STATE_DO_TASK    
    T4 = time.time()
    T3 = T4
    while (T3-T4<10):
        if check_stop():
            with open(file_path, mode='w', encoding='utf-8') as file_obj:
                for i in range(len(current_count)):
                    file_obj.write(str(current_count[i]))
            sys.exit(0)
        
        T3 = time.time()
        side_image = side_camera.read()
        finded_center_x = find_side_center(side_image, current_tsk)
        
        if finded_center_x == -1:  # 检测到好人
            current_count[2] += 1
            T1 = time.time()
            T2 = T1
            while(T2-T1<3) :
                front_image = front_camera.read()
                angle = cruiser.infer_cnn(front_image)
                cart.set_speed(normal_speed)
                cart.steer(angle)
                T2 = time.time()
            return STATE_CRUISE
        front_image = front_camera.read()
        angle = cruiser.infer_cnn(front_image)
        cart.set_speed(slow_speed)
        cart.steer(angle)
        
        if finded_center_x != None:
            # config里面加入x属性值待定
            if task[current_tsk]['x'] < finded_center_x-task[current_tsk]['locate_range']:
                cart.set_speed(slow_speed*cam_dir)
                cart.steer(angle)
            elif task[current_tsk]['x'] > finded_center_x+task[current_tsk]['locate_range']:
                cart.set_speed(slow_speed*cam_dir*(-1))
                cart.steer(angle)
            else:
                cart.stop()
                print("ok")
                sys.stdout.flush()
                return STATE_DO_TASK
    steer_time(3,20)
    return STATE_CRUISE

    # 长时间未到达位置 继续巡航？ return STATE_CRUISE


def do_task_handler():
    global current_tsk
    global cam_dir
    global current_count
    global current_count_once
    if current_tsk == 1:
        purchase(current_tsk)  # 采购货物
        current_count[0] += 1
        current_count_once[0] +=1
    elif current_tsk == 2:
        cultural_exchange()  # 文化交流
        current_count[1] += 1
        current_count_once[1] +=1
    elif current_tsk == 3:
        protect(current_tsk,current_count[2],cam_dir)  # 守护丝路
        current_count[2] += 1
        current_count_once[2] +=1
    elif current_tsk == 4:  
        Singandfriendship()  # 放歌友谊
        current_count[3] += 1
        current_count_once[3] +=1
    elif current_tsk == 5:
        Over_the_mountains()  # 翻山越岭
        current_count[4] += 1    
        current_count_once[4] +=1
    elif current_tsk == 6:
        bartering(current_tsk)  #  以物易物
        current_count[5] += 1    
        current_count_once[5] +=1 
    return STATE_CRUISE


state_map = {
    STATE_IDLE: idle_handler,
    STATE_CRUISE: cruise_handler,
    STATE_LOCATE_TASK: locate_task_handler,
    STATE_DO_TASK: do_task_handler,
}


if __name__ == '__main__':
#    current_count = [0,0,0,0,0,0]
#    start_init()
#    current_state = STATE_IDLE
#    while True:
#        current_state = state_map[current_state]()
    file_path = './task.txt'
    current_count =[0,0,0,0,0,0]
    current_count_once = [0,0,0,0,0,0]
    with open(file_path, mode='r', encoding='utf-8') as file_obj:
        temp = file_obj.read()
        for i in range(len(temp)):
            current_count[i] = int(temp[i])
    start_init()
    print(current_count)
    print(current_count_once)
    current_state = STATE_IDLE
    while check_stop()== False:
#        textlist = os.popen()
#        print(textlist)
#        sys.stdout.flush()
        current_state = state_map[current_state]()
    with open(file_path, mode='w', encoding='utf-8') as file_obj:
        for i in range(len(current_count)):
            file_obj.write(str(current_count[i]))
    front_camera.stop()
    side_camera.stop()
    print("Finished!!")

#    file_path = './task.txt'
#    current_count =[0,0,0,0,0,0]
#    with open(file_path, mode='r', encoding='utf-8') as file_obj:
#        temp = file_obj.read()
#        for i in range(len(temp)):
#            current_count[i] = int(temp[i])
#    start_init()
#    current_state = STATE_IDLE
#    try:
#        while check_stop()== False:
#            textlist = os.popen()
#            print(textlist)
#            sys.stdout.flush()
#            current_state = state_map[current_state]()
#    except(IOError):
#        with open(file_path, mode='w', encoding='utf-8') as file_obj:
#            for i in range(len(current_count)):
#                file_obj.write(str(current_count[i]))
#        startmachine()
#        os.system('reboot')
