#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 地标 -50摄像头右转  125左转   改成 -85 or 85    sign 巡航用
task = {
        1: {"label": "采购货物", "angle": [1],  "sign": 6,  "x": 435, "speed": 15, "time": 0.5, "height1": 7, "height2": 11, "locate_range": 10},
        2: {"label": "文化交流", "angle": [0,0,0],  "sign": 1,  "x": 440, "locate_range": 30},
        3: {"label": "守护丝路", "angle": [0,0,0,1], "sign": 2,  "x": 320,"hit_x":[450,450,450,190], "locate_range": 10},
        4: {"label": "放歌友谊", "angle": [0],  "sign": 4, },
        5: {"label": "翻山越岭", "angle": [0],  "sign": 3,  },
        6: {"label": "以物易物", "angle": [1], "sign": 5, "x":132,"speed":15,"time":0.8,"height1":7,"height2":17, "locate_range": 10},
}


CONTROLLER = "mc601"

REC_NUM = 8     # 图标出现次数而统计确认识别结果


sign_label = {"background": 1, "castle": 1,"friendship": 1, "guard": 1, "purchase": 1, "trade": 1}
hit_label = {"badperson": 1, "badperson2": 2,"goodperson": 3, "goodperson2": 4}
trade_label = {'walnut':1,
        'grape':2,
        'kiwi':3,
        'lichee':4,
        'rice':5,
        'loquat':6,}
