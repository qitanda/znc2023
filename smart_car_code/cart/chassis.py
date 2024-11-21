#!/usr/bin/python3
# -*- coding: utf-8 -*-

import time
import serial
import sys

sys.path.append("../")
from config import CONTROLLER

comma_head_01_motor = bytes.fromhex('77 68 06 00 02 0C 02 01')
comma_head_02_motor = bytes.fromhex('77 68 06 00 02 0C 02 02')
comma_head_03_motor = bytes.fromhex('77 68 06 00 02 0C 02 03')
comma_head_04_motor = bytes.fromhex('77 68 06 00 02 0C 02 04')
comma_head_all_motor_1 = bytes.fromhex('77 68 0c 00 02 7a 01')
comma_head_all_motor_2 = bytes.fromhex('77 68 0c 00 02 7a 02')
comma_trail = bytes.fromhex('0A')


def speed_limit(speeds):
    i = 0
    for speed in speeds:
        if speed > 100:
            speed = 100
        elif speed < -100:
            speed = -100
        speeds[i] = speed
        i = i + 1
    return speeds


class Chassis:

    def __init__(self, ID):
        """
        :rtype: object
        """
        self.speed = 10
        self.kx = 0.85
        portx = "/dev/ttyUSB0"
        if CONTROLLER == "mc601":
            bps = 380400
        elif CONTROLLER == "wobot":
            bps = 115200
        else:
            bps = 115200
        self.serial = serial.Serial(portx, int(bps), timeout=0.000005, parity=serial.PARITY_NONE, stopbits=1)
        self.p = 0.8
        self.slow_ratio = 0.97
        self.min_speed = 20
        if ID == 1:
            self.head = comma_head_all_motor_1
        elif ID == 2:
            self.head = comma_head_all_motor_2

    def steer(self, Speed, angle):
        # print(angle)
        Speed = int(Speed)
        delta = angle * self.kx
        delta_speed = int(delta * Speed)
        print(delta_speed)
        left_front = self.speed + delta_speed
        right_front = self.speed - delta_speed
        left_back = self.speed + delta_speed
        rigth_back = self.speed - delta_speed

        self.move([left_front, right_front, left_back, rigth_back])

    def stop(self):
        self.move([0, 0, 0, 0])

    def move(self, speeds):
        left_front = int(speeds[0])
        right_front = -int(speeds[1])
        left_rear = int(speeds[2])
        right_rear = -int(speeds[3])
        self.min_speed = int(min(speeds))
        left_front_kl = bytes.fromhex('01') + left_front.to_bytes(1, byteorder='big', signed=True)
        right_front_kl = bytes.fromhex('02') + right_front.to_bytes(1, byteorder='big', signed=True)
        left_rear_kl = bytes.fromhex('03') + left_rear.to_bytes(1, byteorder='big', signed=True)
        right_rear_kl = bytes.fromhex('04') + right_rear.to_bytes(1, byteorder='big', signed=True)
        send_data_all_motor = (self.head + left_front_kl + right_front_kl + left_rear_kl + right_rear_kl
                               + comma_trail)
        self.serial.write(send_data_all_motor)
        time.sleep(0.01)

    def reverse(self):
        speed = self.speed
        self.move([-speed, -speed, -speed, -speed])

    def set_speed(self, speed):
        self.speed = speed;


def test():
    c = Chassis()
    while True:
        c.move([-50, 0, -50, 0])
        time.sleep(4)
        c.stop()
        time.sleep(1)
