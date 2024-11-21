import time
import numpy as np
import camera
import cv2
import sys
import navigation
import correspond

front_camera = camera.camera(0)
side_camera = camera.camera(1)


def start_init():
    front_camera.start()
    side_camera.start()
    # grab_servo.servo_control(0, 50)  # 爪子放到初始位置    0闭    40开
    # flag_servo.servo_control(40, 80)  # 旗子放到初始位置
    # camera_servo.servo_control(0, 80)  # 摄像头转到初始位置   175 左  0 右
    # hit_servo.servo_control(100, 80)  # 伸缩结构回到初始位置  0伸开  100收缩  伸出去速度快，收缩速度慢
    # light.light_off()  # light.light_control(0,r,g,b)
    time.sleep(0.5)


def idle_handler():
    print("idle")
    sys.stdout.flush()
    # while True:
    #     if stop_button.clicked():
    #         sys.exit(0)
    #     if start_button.clicked():
    #         return STATE_CRUISE


def cruise_handler():
    global current_tsk
    global cam_dir
    global current_count
    global current_count_once
    sign_list = [0] * 20
    navigation.pid_thread(0)
    while True:
        # if check_stop():
        #     with open(file_path, mode='w', encoding='utf-8') as file_obj:
        #         for i in range(len(current_count)):
        #             file_obj.write(str(current_count[i]))
        #     sys.exit(0)
        front_image = front_camera.read()
        angle, zhongxian, c = navigation.XunX(front_image)
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
                        if current_count[current_tsk - 1] > current_count_once[current_tsk - 1]:
                            current_count_once[current_tsk - 1] += 1
                            return STATE_CRUISE
                        else:
                            if task[current_tsk]['angle'][
                                current_count[current_tsk - 1]] == 175:  # ?待定:     摄像头方向 右边-1 左边1   175左 0右
                                cam_dir = 1
                            else:
                                cam_dir = -1
                        camera_servo.servo_control(task[current_tsk]['angle'][current_count[current_tsk - 1]], 80)
                        lift_height(4, 80)
                        return STATE_LOCATE_TASK


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
    current_count = [0, 0, 0, 0, 0, 0]
    current_count_once = [0, 0, 0, 0, 0, 0]
    with open(file_path, mode='r', encoding='utf-8') as file_obj:
        temp = file_obj.read()
        for i in range(len(temp)):
            current_count[i] = int(temp[i])
    start_init()
    print(current_count)
    print(current_count_once)
    current_state = STATE_IDLE
    # while check_stop()== False:
    #        textlist = os.popen()
    #        print(textlist)
    #        sys.stdout.flush()
    # current_state = state_map[current_state]()
    with open(file_path, mode='w', encoding='utf-8') as file_obj:
        for i in range(len(current_count)):
            file_obj.write(str(current_count[i]))
    front_camera.stop()
    side_camera.stop()
    print("Finished!!")
