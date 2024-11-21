# 从视频文件中截取图片

import cv2
import os

# 视频路径
video_name = "1.mp4"
# 图片保存路径
img_dir = "images"
# 截取图片的间隔，单位毫秒
t = 500

vidcap = cv2.VideoCapture(video_name)

# 视频文件的当前位置（以毫秒为单位）
frame_time = 0
for i in range(5000):
    if not os.path.exists(img_dir):
        os.makedirs(img_dir)
    img_name = "data" + str(i) + ".jpg"
    vidcap.set(cv2.CAP_PROP_POS_MSEC, frame_time)
    success, image = vidcap.read()
    if success:
        cv2.imwrite(img_dir + "/" + img_name, image)  
        # cv2.imshow("frame%s" % frame_time, image)
        # cv2.waitKey()
        print("save ",img_name)
    else:
        break
    frame_time += t
