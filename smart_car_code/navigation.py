import cv2
import numpy
import numpy as np
import time
import pid
import correspond
import threading
import math


def nothing(*arg):
    pass

# 480*640
icol = (18, 0, 196, 36, 255, 255)
last_angle = 0
last_zhongxian = 0
path = "test/9.jpg"
# Show the original image.
# frame = path + str(33) + '.jpg'
frame = cv2.imread(path)
# cv2.imshow('frame', frame)
# cv2.waitKey(1000)
frame = cv2.resize(frame, (640, 480))
# l = [16, 45, 65]  # [17, 55, 128]#阈值
# h = [44, 255, 255]  # [24, 255, 255]#阈值
frame_x = 128
frame_y = 96
image_flag = 1
video_flag = 0
print_flag = 1
flag_done = 0
distinct_threshold = 4  # 640*480:30;128*96:6


def zh_ch(string):
    return string.encode("gbk").decode('UTF-8', errors='ignore')


# mm = 320
# cap = cv2.VideoCapture(0)
def XunX(img):
    # cv2.imshow('frame', img)
    # Blur methods available, comment or uncomment to try different blur methods.
    start2 = time.time()
    global last_angle, last_zhongxian
    start = time.time()
    frameBGR = cv2.GaussianBlur(img, (7, 7), 0)
    hsv = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2HSV)
    # HSV values to define a colour range.
    colorLow = numpy.array([35, 43, 0])  # [35, 43, 46]
    colorHigh = numpy.array([77, 225, 255])  # [77, 225, 225]
    mask = cv2.inRange(hsv, colorLow, colorHigh)
    # Show the first mask
    # if image_flag:
    #     cv2.imshow('mask-plain', mask)
    kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    start5 = time.time()
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)
    img = cv2.resize(img, (128, 96))
    mask = cv2.resize(mask, (128, 96))
    end5 = time.time()
    # print("time5 cost", end5-start5, "s")
    if image_flag:
        cv2.imshow('mask-plain', mask)
    midline = int(frame_x / 2)
    midline_l = int(frame_x / 2)
    midline_r = int(frame_x / 2)
    left = np.array([])
    right = np.array([])
    mid = np.array([])
    sum_count = 0
    shizi = 0
    huandao = 0
    flag = 0
    area = 0
    end2 = time.time()
    # print("time2 cost", end2-start2, "s")
    start4 = time.time()
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    if contours:
        for i in range(0, len(contours), 1):
            area += cv2.contourArea(contours[i])
    end4 = time.time()
    # print("area:", area)
    # print("time4 cost", end4-start4,"s")
    start1 = time.time()
    if area >= 1300:
        lines = cv2.HoughLinesP(mask, 1, np.pi / 180, 30, minLineLength=20, maxLineGap=15)
        flag_l = flag_r = flag_h = 0
        line_count1 = line_count2 = angle1 = base_x1 = base_x2 = angle_sum1 = angle_sum2 = 0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 1)
            angle1 = math.atan2(y2 - y1, x2 - x1)
            if angle1 < 0: angle1 += np.pi
            angle1 = np.pi - angle1
            if np.pi/3 < angle1 < np.pi/2:
                flag_l = 1
                line_count1 +=1
                base_x1 += (x2 - x1) / (y2 - y1) * (frame_y - 1 - y1) + x1
                angle_sum1 += angle1
            if np.pi/2 < angle1 < 2 * np.pi/3:
                flag_r = 1
                line_count2 += 1
                base_x2 += (x2 - x1) / (y2 - y1) * (frame_y - 1 - y1) + x1
                angle_sum2 += angle1
            if angle1 < np.pi/4 or angle1 > 3*np.pi/4: flag_h = 1
        # print("flag_l", flag_l)
        # print("flag_r", flag_r)
        # print("flag_h", flag_h)
        if flag_l and flag_h and flag_r:
            base_x1 /= line_count1
            base_x2 /= line_count2
            angle_sum1 /= line_count1
            angle_sum2 /= line_count2
            shizi = 1
            last_angle = (angle_sum1 + angle_sum2)/2
            last_zhongxian = (base_x1 + base_x2)/2
            if print_flag:
                print("shizi:",shizi)
            c = cv2.waitKey(1)
            return last_angle, last_zhongxian, c
    end1 = time.time()
    # print("time1 cost", end1 - start1, 's')
    if 0:#550
        huandao = 1
        for i in range(frame_y - 1, int(frame_y / 2), -1):
            mid = np.append(mid, frame_x - (i - frame_y / 2) / (frame_y / 2) * frame_x / 2)
            if image_flag:
                cv2.circle(img, (int(mid[frame_y - i - 1]), i), 1, (0, 0, 255), 0)
    # print(sum_count)
    # print(huandao)
    # if shizi: return mid
    # if huandao: return mid
    start3 = time.time()
    if shizi == 0 and huandao == 0:
        #for i in range(0, len(contours), 1):

        for i in range(frame_y - 1, int(frame_y / 2), -1):
            for j in range(midline_l - 1, 0, -1):
                if (mask[i][j] == 0) and (mask[i][j - 1] == 255):
                    if i < frame_y - 1 and abs(j - left[frame_y - i - 2]) < distinct_threshold:
                        left = np.append(left, j)
                        if image_flag:
                            cv2.circle(img, (j, i), 1, (255, 0, 255), 0)
                        break
                    elif i < frame_y - 1 and abs(j - left[frame_y - i - 2]) >= distinct_threshold:
                        # if i >= frame_y - 3:
                        if left[frame_y - i - 2] > 1:
                            left = np.append(left, left[frame_y - i - 2])
                        else:
                            left = np.append(left, j)
                        # else:
                        #     left = np.append(left, 3/2*left[frame_y - i - 2] - 1/2*left[frame_y - i - 4])
                        if image_flag:
                            cv2.circle(img, (int(left[frame_y - i - 2]), i), 1, (255, 0, 255), 0)
                        break
                    elif i == frame_y - 1:
                        left = np.append(left, j)
                        if image_flag:
                            cv2.circle(img, (j, i), 1, (255, 0, 255), 0)
                        break
                elif j == 1:
                    if i < frame_y - 1 and abs(j - left[frame_y - i - 2]) < distinct_threshold:
                        left = np.append(left, j)
                        if image_flag:
                            cv2.circle(img, (j, i), 1, (255, 0, 255), 0)
                        break
                    elif i < frame_y - 1 and abs(j - left[frame_y - i - 2]) >= distinct_threshold:
                        left = np.append(left, left[frame_y - i - 2])
                        if image_flag:
                            cv2.circle(img, (int(left[frame_y - i - 2]), i), 1, (255, 0, 255), 0)
                        break
                    elif i == frame_y - 1:
                        left = np.append(left, j)
                        if image_flag:
                            cv2.circle(img, (j, i), 1, (255, 0, 255), 0)
                        break
                        # left = np.append(left, j)
                    # left1 = j
                    # cv2.circle(img, (int(left[frame_y - i - 1]), i), 1, (255, 0, 255), 0)
            for j in range(midline_r, frame_x - 1, 1):
                if (mask[i][j] == 0) and (mask[i][j + 1] == 255):
                    if i < frame_y - 1 and abs(j - right[frame_y - i - 2]) < distinct_threshold:
                        right = np.append(right, j)
                        if image_flag:
                            cv2.circle(img, (j, i), 1, (255, 0, 255), 0)
                        break
                    elif i < frame_y - 1 and abs(j - right[frame_y - i - 2]) >= distinct_threshold:
                        if right[frame_y - i - 2] < frame_x - 2:
                            right = np.append(right, right[frame_y - i - 2])
                        else:
                            right = np.append(right, j)
                        # else:
                        # right = np.append(right, 3/2*right[frame_y - i - 2] - 1/2*right[frame_y - i - 4])
                        if image_flag:
                            cv2.circle(img, (int(right[frame_y - i - 2]), i), 1, (255, 0, 255), 0)
                        break
                    elif i == frame_y - 1:
                        right = np.append(right, j)
                        if image_flag:
                            cv2.circle(img, (j, i), 1, (255, 0, 255), 0)
                        break
                elif j == frame_x - 2:
                    if i < frame_y - 1 and abs(j - right[frame_y - i - 2]) < distinct_threshold:
                        right = np.append(right, j)
                        if image_flag:
                            cv2.circle(img, (j, i), 1, (255, 0, 255), 0)
                        break
                    elif i < frame_y - 1 and abs(j - right[frame_y - i - 2]) >= distinct_threshold:
                        right = np.append(right, right[frame_y - i - 2])
                        if image_flag:
                            cv2.circle(img, (int(right[frame_y - i - 2]), i), 1, (255, 0, 255), 0)
                        break
                    elif i == frame_y - 1:
                        right = np.append(right, j)
                        if image_flag:
                            cv2.circle(img, (j, i), 1, (255, 0, 255), 0)
                        break
            midline = int((left[frame_y - i - 1] + right[frame_y - i - 1]) / 2)
            midline_l = int(left[frame_y - i - 1]) + 10
            midline_r = int(right[frame_y - i - 1]) - 10
        # for i in range
        for i in range(frame_y - 1, int(frame_y / 2), -1):
            if left[frame_y - i - 1] != 1 and right[frame_y - i - 1] != frame_x - 2:
                mid = np.append(mid, (left[frame_y - i - 1] + right[frame_y - i - 1]) / 2)
            elif left[frame_y - i - 1] == 1 and right[frame_y - i - 1] != frame_x - 2:
                mid = np.append(mid, right[frame_y - i - 1] - (0.2 + i / frame_y * 0.30) * frame_x)
            elif left[frame_y - i - 1] != 1 and right[frame_y - i - 1] == frame_x - 2:
                mid = np.append(mid, left[frame_y - i - 1] + (0.2 + i / frame_y * 0.30) * frame_x)
            else:
                mid = np.append(mid, frame_x / 2)  # 0.32;0.18
            if image_flag:
                cv2.circle(img, (int(mid[frame_y - i - 1]), i), 1, (0, 0, 255), 0)
    end3 = time.time()
    # print('time cost3', end3 - start3, 's')
    end = time.time()
    # print('time cost', end - start, 's')
    gray_img = np.zeros((96, 128, 3), np.uint8)
    gray_img = cv2.cvtColor(gray_img, cv2.COLOR_BGR2GRAY)
    for i in range(frame_y - 1, int(frame_y / 2), -1):
        cv2.circle(gray_img, (int(mid[frame_y - i - 1]), i), 1, (255, 255, 255), 0)
    if image_flag:
        cv2.imshow('mask-hough', gray_img)
    # lines = cv2.HoughLines(gray_img, 1, np.pi / 180, 30)
    #start1 = time.time()
    lines = cv2.HoughLinesP(gray_img, 1, np.pi / 180, 30, minLineLength=15, maxLineGap=15)
    #end1 = time.time()
    #print("time1 cost", end1 - start1, 's')
    line_count = 0
    angle_sum = 0
    base_x = 0
    # if all(lines == [[]]) == 0:
    # if lines == np.none:
    if lines is not None:
        # print(lines)
        # for line in lines:
        #     line_count += 1
        #     rho = line[0][0]
        #     theta = line[0][1]
        #     a = np.cos(theta)
        #     b = np.sin(theta)
        #     x0 = a * rho
        #     y0 = b * rho
        #     x1 = int(x0 + 1000 * (-b))
        #     y1 = int(y0 + 1000 * (a))
        #     x2 = int(x0 - 1000 * (-b))
        #     y2 = int(y0 - 1000 * (a))
        #     cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #     angle = math.atan2(y2 - y1, x2 - x1)
        #     base_x += (x2-x1)/(y2-y1)*(frame_y-1-y0)+x0
        #     if angle < 0: angle += np.pi
        #     angle = np.pi - angle
        #     angle_sum += angle
        #     # print('angle:', angle)
        #     # print((x1,y1),(x2,y2))
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if image_flag:
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 1)
            angle = math.atan2(y2 - y1, x2 - x1)
            base_x += (x2-x1)/(y2-y1)*(frame_y-1-y1)+x1
            if angle < 0: angle += np.pi
            angle = np.pi - angle
            angle_sum += angle
            line_count += 1
    else:
        angle_sum = last_angle
        base_x = last_zhongxian
    if line_count != 0:
        angle_sum /= line_count
        base_x /= line_count
        last_zhongxian = base_x
        last_angle = angle_sum
    # print('angle:', angle_sum)
    # print('x:', base_x)
    if image_flag:
        cv2.imshow('frame', img)
    if video_flag:
        c = cv2.waitKey(1)
    else:
        c = cv2.waitKey(1000)
    return angle_sum,base_x,c


def pid_thread(a):
    if flag_done:
        return 0
    t=threading.Timer(0.03,pid_thread,(a,))
    now_val_y = pid_y.cmd_pid()
    now_val_angular = pid_angular.cmd_pid()
    if video_flag:
        correspond.M_send(270, int(8*(64 - now_val_y)), -int(500*(np.pi/2-now_val_angular)))
    if print_flag:
        print(-int(200*(np.pi/2-now_val_angular)))
        print(int(4*(64-now_val_y)))
    t.start()


mm = int(frame_x / 2)  # 扫线开始的坐标
if video_flag:
    cap = cv2.VideoCapture(1)
# cap = cv2.VideoCapture("/home/zhuji/znc_ws/Video/test_bright.avi")#读取视频
global command_key
pid_y = pid.Pid(frame_x / 2, frame_x / 2, 0.1, 0.15, 0.1)
pid_angular = pid.Pid(np.pi/2, np.pi/2, 0.1, 0.15, 0.1)
start = 0
while True:
    if video_flag:
        test, frame1 = cap.read()
        angle, zhongxian, c = XunX(frame1)
    else:
        angle, zhongxian, c = XunX(frame)
    # aver_y = 0
    # for i in range(0,np.size(zhongxian),1):
    #     aver_y += int(zhongxian[i])/int(np.size(zhongxian))
    # print(aver_y)
    if print_flag:
        print('angle:', angle)
        print('x:', zhongxian)
    pid_angular = pid.Pid(np.pi/2, angle, 0.1, 0.15, 0.1)
    pid_y = pid.Pid(frame_x / 2, zhongxian, 0.05, 0.15, 0.1)
    if start == 0:
        start = 1
        # pid_thread(0)
    if c == 27:
        break
flag_done = 1
pid_y = pid.Pid(frame_x / 2, frame_x / 2, 0.1, 0.15, 0.1)
pid_angular = pid.Pid(np.pi/2, np.pi/2, 0.1, 0.15, 0.1)
while 1:
    correspond.M_send(0, 0, 0)
    time.sleep(1)

def navigation(forward = 1, done = 0):
    if video_flag:
        test, frame1 = cap.read()
        angle, zhongxian, c = XunX(frame1)
    else:
        angle, zhongxian, c = XunX(frame)
    if print_flag:
        print('angle:', angle)
        print('x:', zhongxian)
    pid_angular = pid.Pid(np.pi/2, angle, 0.1, 0.15, 0.1)
    pid_y = pid.Pid(frame_x / 2, zhongxian, 0.05, 0.15, 0.1)
    if start == 0:
        start = 1
        # pid_thread(0)