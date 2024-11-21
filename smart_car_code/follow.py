#!/usr/bin/env python3
#-*- coding:utf-8 -*-
import sys
import cv2
import numpy as np
from PIL import Image as PImage
import time
import math
from math import *
import rospy
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import  Image, Joy
from motion_control import *
from image_ros_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import  Twist
def Get_Hist(img):

    """
        计算直方图数组
    """
    img = np.sum(img, axis=0)
    img.reshape(img.size, 1)

    hist = np.int32(np.around(Normalize(img, 255)))
    # print(hist.shape)
    img = np.zeros((hist.size, hist.size, 3))  # 创建用于绘制直方图的全0图像

    bins = np.arange(hist.size).reshape(hist.size, 1)  # 直方图中各bin的顶点位置

    pts = np.column_stack((bins, hist))
    cv2.polylines(img, [pts], False, (0, 255, 0))
    img = np.flipud(img)
    return img, hist

def Normalize(data, scale):
    """
    归一化-->[0.0, 1.0]
    :param data:
    :param scale:
    :return: [-0.5, 0.5]*scale
    """
    mx = max(data)
    mn = min(data)
    m = (mx - mn)/2 + mn

    return [((float(i) - m) / (mx - mn)+0.5)*scale for i in data]

def get_yellow_lane_bin_img(frame, low_rh, high_rh,low_gs, high_gs,low_bv, high_bv):    # 提取黄色车道线
    #输入原图,返回缩放后的原图与车道线二值图
   
    try:
        lower_array = np.array([low_rh,low_gs,low_bv])   # 第二个参数调节亮度
        upper_array = np.array([high_rh,high_gs, high_bv])
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)     #使用HSV进行车道线识别
        mask = cv2.inRange(hsv, lowerb=lower_array, upperb=upper_array)

        __img = PImage.fromarray(mask)  
        _img = np.array(__img)

        small_img = np.zeros((120, 120))  # 创建全0图像
        small_img = cv2.resize(_img, (120, 120), small_img, cv2.INTER_AREA)

        small_img = small_img.astype(np.uint8)
        retval, bin_img = cv2.threshold(small_img, 125, 255, cv2.THRESH_BINARY)

        small_img = small_img.astype(np.float32)
        img_3chanel = cv2.cvtColor(small_img, cv2.COLOR_GRAY2BGR)
        origin_img = np.zeros((120, 120, 3))  # 创建全0图像
        origin_img = cv2.resize(frame, (120, 120),  origin_img, cv2.INTER_AREA)
       # cv2.imshow("try", mask)
       # cv2.waitKey(10)
        return origin_img, bin_img
    except:
        s = sys.exc_info()
        print("Error '%s' happened on line %d" % (s[1], s[2].tb_lineno))

class CONTROLER:
    def __init__(self):
        #PID相关参数
        self.target_point = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_control_time = time.time() * 1000
        self.current_time = self.last_control_time
        self.last_error = 0.0        
        self.P = 1.0
        self.I = 1.5
        self.D = 0.0
        self.pid_out_rate = 0.0000000000007
        self.pid_out_power =7
        self.windup_guard = 20.0

        #图像相关
        self.RC_buff_length = 5
        self.history_left_lane = [0] * self.RC_buff_length  # 惯性滤波缓存区
        self.history_right_lane = [0] * self.RC_buff_length  # 惯性滤波缓存区
        self.history_angle = [float(0.0)] * self.RC_buff_length  # 惯性滤波缓存区

        #颜色阈值

        self.high_h = 70
        self.high_s = 255
        self.high_v = 255
        self.low_h  = 37
        self.low_s  = 90
        self.low_v  = 70

        self.mn_width = 5.0
        self.mn_height =130.0
        self.mn_area = 1300.0
        self.mn_distance =40.0
        self.Frame_image = None
        #巡线相关
        self.camera_correction_done = False     # 摄像头校准标志
        self.crossroads = False                 # 十字路口标志
        self.last_direction = 0                 # 上次转弯的方向,debug用
        self.follow_type = True                 #巡线开关
        self.first_stop =True
        self.vo_flag_1=False
        self.vo_flag_2=False
        self.vo_flag_3=False

        #速度相关
        self.max_speed = 0.16
        self.min_speed = 0.08
        self.sign_speed = self.max_speed
        self.vel = Twist()
        self.pid_cmd_vel_x=0
        self.pid_cmd_vel_z=0
        self.scale_1 = 3
        self.scale_2 = 3
        #ros相关

        rospy.init_node('follow_line', anonymous=False)
        self.cam_sub=rospy.Subscriber("/usb_cam/image_raw", Image, self.Image_callback)      #获取图像
        self.follow_sub=rospy.Subscriber('/follow_type', Int32, self.follow_type_callback )     #获取巡线状态
        self.vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1000)      #发布速度
        rospy.Subscriber('/odom', Odometry, self.get_odom)
        self.coor =motion_control()
        rospy.on_shutdown(self.cancel)


    def cancel(self):
        self.send_VelOrder(0,0,0)

        self.cam_sub.unregister()
        self.follow_sub.unregister()
        self.vel_pub.unregister()

    def fliter_left_right_bias(self, hist_array):
        """
            滤波器函数，过滤直方图中的干扰
            :param hist_array:        直方图数组
            :return:脉冲数量，最左边的脉冲峰顶的索引位置，最右边脉冲峰顶的索引位置，过滤后的图像波形
        """
        try:
            left_distance = right_distance = 0
            shape = list(hist_array)
            # 计算脉冲个数
            pulse_flag = 0  # 波形连续标志
            pulse_height = []
            pulse_index = []
            # 1 对直方图进行波形切割，记录了波峰的高度和索引
            for i in range(len(shape)):
                if pulse_flag and (shape[i] > 50 and i < len(shape) - 1):
                    _pulse_height.append(shape[i])
                    _pulse_index.append(i)
                elif pulse_flag and ((shape[i] <= 50) or (i == (len(shape) - 1))):
                    pulse_flag = 0  # 波形分割
                    pulse_height.append(_pulse_height)
                    pulse_index.append(_pulse_index)
                elif shape[i] > 0:
                    pulse_flag = 1
                    _pulse_height = []
                    _pulse_index = []
                    _pulse_height.append(shape[i])
                    _pulse_index.append(i)

            pulse_point_preprocess = []
            pulse_height_preprocess = []
            # 2 根据脉冲高度和面积，过滤掉干扰脉冲
            for i, points in enumerate(pulse_height):
                height = max(points)
                area = sum(points)
                width = len(points)
                if (width >= self.mn_width and (area >= self.mn_area or height >= self.mn_height)):
                    pulse_height_preprocess.append(points)
                    pulse_point_preprocess.append(pulse_index[i])
            # 对过滤后的脉冲波形进行绘图
            
            __array = np.array([0] * len(shape))
            for points in pulse_point_preprocess:
                for ind in points:
                    __array[ind] = shape[ind]
            __array.reshape((1, len(shape)))
            img = np.zeros((len(shape), len(shape), 3))  # 创建用于绘制直方图的全0图像
            bins = np.arange(len(shape)).reshape(len(shape), 1)  # 作为索引，0~len()
            pts = np.column_stack((bins, __array))
            cv2.polylines(img, [pts], False, (0, 255, 0))
            hist_img = np.flipud(img)   # 生成滤波后的波形图
        
            # 3 对于过滤后的脉冲进行左右双峰的定位
            if len(pulse_point_preprocess) == 1:  # 单峰
                points = pulse_height_preprocess[0]
                mx_h = max(points)
                mx_h_ind = points.index(mx_h)
                left_distance = right_distance = pulse_point_preprocess[0][mx_h_ind]

            elif len(pulse_point_preprocess) == 2:  # 双峰
                # 方法1：取峰顶最高点
                points = pulse_height_preprocess[0]
               
                mx_h = max(points)
                mx_h_ind = points.index(mx_h)
                left_distance = pulse_point_preprocess[0][mx_h_ind]
                
                points = pulse_height_preprocess[-1]
                mx_h = max(points)
                mx_h_ind = points.index(mx_h)
                right_distance = pulse_point_preprocess[-1][mx_h_ind]
                # 方法2：取峰顶中心点
                

            elif len(pulse_point_preprocess) > 2:  # 多峰，从左向右，从右向左取最高的2个
                left_ind = 0
                right_ind = len(pulse_point_preprocess) - 1
                left_mx = []
                left_mx_ind = []
                right_mx = []
                right_mx_ind = []
                while left_ind < right_ind:
                    points = pulse_height_preprocess[left_ind]
                    if sum(points) > sum(left_mx):
                        left_mx = points
                        left_mx_ind = pulse_point_preprocess[left_ind]
                    points = pulse_height_preprocess[right_ind]
                    if sum(points) > sum(right_mx):
                        right_mx = points
                        right_mx_ind = pulse_point_preprocess[right_ind]
                    left_ind += 1
                    right_ind -= 1
                    pass
                if left_ind == right_ind:
                    points = pulse_height_preprocess[right_ind]
                    if sum(points) > sum(right_mx):
                        right_mx = points
                        right_mx_ind = pulse_point_preprocess[right_ind]

                # 多峰变双峰
                # 方法2 求左侧最高峰的中心点，右侧最高峰的中心点
                left_distance = int(sum(left_mx_ind) / len(left_mx_ind))
                right_distance = int(sum(right_mx_ind) / len(right_mx_ind))

            return len(pulse_point_preprocess), left_distance, right_distance, hist_img

        except:
            s = sys.exc_info()
            print("Error '%s' happened on line %d" % (s[1], s[2].tb_lineno))

    def get_servo_angle(self):
        try:
            """
            :return:舵机数值
            """
            self.output = self.output ** int(self.pid_out_power)
            angle = self.pid_out_rate * self.output
            #print("pid_power_out=", self.output, "angle=", angle)
            return angle
        except:
            s = sys.exc_info()
            print("Error '%s' happened on line %d" % (s[1], s[2].tb_lineno))

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        """
        error = self.target_point - feedback_value
        error = error
        self.current_time = time.time() * 1000  # ms
        delta_time = self.current_time - self.last_control_time
        delta_error = error - self.last_error

        self.PTerm = self.P * error
        self.ITerm += error * delta_time

        # 超调震荡窗口
        if (self.ITerm < -self.windup_guard):
            self.ITerm = -self.windup_guard
        elif (self.ITerm > self.windup_guard):
            self.ITerm = self.windup_guard

        self.DTerm = 0.0
        if delta_time > 0:
            self.DTerm = delta_error / delta_time

        # Remember last time and last error for next calculation
        self.last_time = self.current_time
        self.last_error = error

        self.output = self.PTerm + (self.I * self.ITerm) + (self.D * self.DTerm)

    def put_left_lane(self, data):
        for i in range(self.RC_buff_length - 1):
            self.history_left_lane[i] = self.history_left_lane[i + 1]
        self.history_left_lane[self.RC_buff_length - 1] = data

    def put_right_lane(self, data):
        for i in range(self.RC_buff_length - 1):
            self.history_right_lane[i] = self.history_right_lane[i + 1]
        self.history_right_lane[self.RC_buff_length - 1] = data

    def put_angle(self, data):
        for i in range(len(self.history_angle)-1):
            self.history_angle[i] = self.history_angle[i + 1]
        self.history_angle[-1] = data

    def get_history_turn(self):
        # 舵机角度<0为左转
        angle = sum(self.history_angle) / len(self.history_angle)
        if angle < 0:
            return 1
        elif angle > 0:
            return 2
        else:
            return 0
      
    def get_history_angle(self):
        return sum(self.history_angle) / len(self.history_angle)

    def get_left_lane_history(self):
        return sum(self.history_left_lane[:self.RC_buff_length])

    def get_right_lane_history(self):
        return sum(self.history_right_lane[:self.RC_buff_length])

    def call_back_30hz(self, cv_image):
        try:
            line_low = 0.99
            line_up = 0.85
            if self.crossroads and self.follow_type:
                self.send_VelOrder(self.sign_speed,0,0)
                time.sleep(2.5)
                self.crossroads =False

            if not self.camera_correction_done:
                # 相机校准过程中，图像不缩放，画出中线以供校准
                H = cv_image.shape[0]
                W = cv_image.shape[1]
                cv2.line(cv_image, pt1=(0, int(H * line_low)), pt2=(W, int(H * line_low)), color=(0, 255, 0), thickness=2)
                cv2.line(cv_image, pt1=(0, int(H * line_up)), pt2=(W, int(H * line_up)), color=(0, 255, 0), thickness=2)
                cv2.line(cv_image, pt1=(W // 2, 0), pt2=(W // 2, H), color=(0, 255, 0), thickness=2)
                cv2.imshow("lanes", cv_image)
                if  cv2.waitKey(1) == 32:
                    self.camera_correction_done = True
                    cv2.destroyAllWindows()
            else:
                # 正常行驶时，缩放，显示滤波图像
                origin_img, bin_img = get_yellow_lane_bin_img(cv_image, self.low_h, self.high_h,self.low_s, self.high_s,self.low_v, self.high_v)      # 获取缩放后的原图与二值图
                H = origin_img.shape[0]
                W = origin_img.shape[1]
                #cv2.imshow("test", origin_img)
                #cv2.waitKey(10)
                bin_img_rectangle_ROI = bin_img[int(H * line_up):int(H * line_low), :]     # 获取二值图的ROI区域
                hist_bin_line_img, hist_array = Get_Hist(bin_img_rectangle_ROI[:, :])    # 计算直方矩阵，以及脉冲波形图
                line_count, left, right, hist_bin_line_img_filtered = self.fliter_left_right_bias(hist_array)    # 对直方图进行滤波
                mid_line = int(len(hist_array)/2)  # 视野中线
                # 根据线的数量决定PID的输入量
                #print(line_count,left,right)
                if line_count >= 3:  # 十字路口
                        self.crossroads = True
                else:
                        self.crossroads = False



                if self.vo_flag_1:                     #当识别到漩涡后，强制循迹走直线
                    if line_count<2 :
                        line_count = 2 
                        right= 104
                    cur_coordinate_x = self.orientationx-self.start_x    
                    cur_coordinate_y = self.orientationy-self.start_y
                    distance = sqrt (cur_coordinate_x*cur_coordinate_x+cur_coordinate_y*cur_coordinate_y)
                    if distance >0.9:
                        self.follow_type=0
                        self.coor.coordinate(0,0,-90)
                        self.coor.coordinate(0.3,0,0)
                        self.follow_type=1
                        self.vo_flag_1=False
                      
                if self.vo_flag_2:
                    if line_count<2 :
                        line_count = 2 
                        right= 104
                    cur_coordinate_x = self.orientationx-self.start_x    
                    cur_coordinate_y = self.orientationy-self.start_y
                    distance = sqrt (cur_coordinate_x*cur_coordinate_x+cur_coordinate_y*cur_coordinate_y)
                    if distance >0.82:
                        self.follow_type=0
                        self.coor.coordinate(0,0,-90)
                        self.coor.coordinate(0.3,0,0)
                        self.follow_type=1
                        self.vo_flag_2=False

                if self.vo_flag_3:
                    if line_count<2 :
                        line_count = 2 
                        right= 104
                    cur_coordinate_x = self.orientationx-self.start_x    
                    cur_coordinate_y = self.orientationy-self.start_y
                    distance = sqrt (cur_coordinate_x*cur_coordinate_x+cur_coordinate_y*cur_coordinate_y)
                    if distance >1.75:
                        self.follow_type=0
                        self.coor.coordinate(0,0,-90)
                        self.vo_flag_3=False

                if line_count >= 2:
                    if ((right - left) >= self.mn_distance):
                        bais = mid_line - (right + left) // 2
                        #print("检测到多线")
                        #print("bais>0时应左转，bais<0时应右转, bais=%s", bais)
                        self.update(bais)
                        angle_value = self.get_servo_angle()*self.scale_1
                        self.put_angle(angle_value)

                        self.put_left_lane(1)
                        self.put_right_lane(1)
                    else:
                        #print("检测到多线，但左右线距过窄，延续之前的角度")
                        angle_value = self.get_history_angle()  # 延续之前的角度
                        self.put_angle(angle_value)

          
                if line_count < 2:
                    # 左右车道线惯性滤波
                    l = self.get_left_lane_history()
                    r = self.get_right_lane_history()

                    if l and r:
                        angle_value = self.get_history_angle()*2  # 延续之前的角度
                        self.put_angle(angle_value)
                        if left < mid_line:
                            self.put_left_lane(1)
                            self.put_right_lane(0)

                        if left > mid_line:
                            self.put_left_lane(0)
                            self.put_right_lane(1)
                      
                    elif l or r:
                        # 单线情况，跟随之前时刻的转向
                        if self.get_history_turn() == 1:  # 前一个时刻是左转
                            #print("前一个时刻是左转")
                            bais =0.5 * (len(hist_array) - right) 
                            self.update(bais)
                            angle_value = self.get_servo_angle()*self.scale_2
                            self.put_angle(angle_value)
                        elif self.get_history_turn() == 2:
                            bais = 0.5 * (0 - left)
                            self.update(bais)
                            angle_value = self.get_servo_angle()*self.scale_2
                            self.put_angle(angle_value)

                        else:
                            # 之前没有转向，说明只是开机时出现的无效单线
                            #print("之前没有转向，说明只是开机时出现的无效单线")
                            angle_value = 0.0
                            self.put_angle(angle_value)
                            # print("line_count=", line_count, "left=", left, "right=", right)

                        # 维持以前的状态
                        if l:
                            self.put_left_lane(1)
                            self.put_right_lane(0)
                        else:
                            self.put_left_lane(0)
                            self.put_right_lane(1)

                    else:
                        # 惯性滤波结果是0
                        self.put_left_lane(0)
                        self.put_right_lane(0)
                        angle_value = 0.0
                        self.put_angle(angle_value)

                angle_value = self.get_history_angle()  # 取惯性滤波后的角度值
                if angle_value>1 :angle_value=1
                if angle_value <-1:angle_value=-1
                #print(angle_value)
                
                # 正数右转，负数左转
                self.last_direction  = pid.get_history_turn()
                self.pid_cmd_vel_x   =  self.sign_speed 
                self.pid_cmd_vel_z   =  -angle_value

                if self.follow_type : 
                    self.send_VelOrder(self.pid_cmd_vel_x,0,self.pid_cmd_vel_z)
                    self.first_stop =True
                if self.follow_type ==0:
                    if self.first_stop: 
                        self.send_VelOrder(0,0,0)
                        time.sleep(0.5)
                        self.send_VelOrder(0,0,0)
                        self.first_stop = False
                    pass

        except:
            s = sys.exc_info()
            print("Error '%s' happened on line %d" % (s[1], s[2].tb_lineno))

    
    def follow_type_callback(self,data):
        if data.data == 3:
            self.sign_speed =self.min_speed               #慢速巡线
            data.data = 1

        if data.data == 4:                                #快速巡线
            self.sign_speed =self.max_speed
            data.data = 1

        if data.data == 5:                                #第一次识别漩涡
            self.sign_speed =0.15
            self.vo_flag_1 = True
            self.start_x=self.orientationx
            self.start_y=self.orientationy
            data.data = 1

        if data.data == 6:                                 #第一次识别漩涡
            self.sign_speed =0.15
            self.vo_flag_2= True
            self.start_x=self.orientationx
            self.start_y=self.orientationy
            data.data = 1

        if data.data == 7:                                 #第一次识别漩涡
            self.sign_speed =0.15
            self.vo_flag_3 = True
            self.start_x=self.orientationx
            self.start_y=self.orientationy
            data.data = 1
        self.follow_type=data.data

    def send_VelOrder(self,v_x,v_y,v_w):      
            self.vel.linear.x = v_x
            self.vel.linear.y = v_y
            self.vel.angular.z = v_w
            self.vel_pub.publish(self.vel)

    def Image_callback(self, image):
        if not isinstance(image, Image): return
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)
        self.Frame_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
    
    def main(self):
        while not rospy.is_shutdown():
            if not isinstance(self.Frame_image, np.ndarray): 
                continue 

            self.call_back_30hz(self.Frame_image)



if __name__=="__main__":

      pid = CONTROLER()
      pid.main()
      print("done")
