# 调用摄像头采集图片，p拍摄，q退出，1~9、a~b切换保存目录

import cv2
import sys
import time
import os

START_NUM = 0

# 摄像头编号
cam = 1

# 目标检测类别
LABEL = ["tower", "hhl", "yyl", "twg", "barge", "trade", "konjac", "citrus", "swordfish", "tornado", "spray", "dam", "vortex"]

print("loading camera ...")
camera = cv2.VideoCapture(cam)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
counter = START_NUM

root = "images"
label_dirs = LABEL
num = 0
length = 12

print("check dir ...")
if not os.path.exists(root):
    os.makedirs(root)
assert os.path.exists(root), "不存在%s目录" % root
for label_dir in label_dirs:
    path = os.path.join(root, label_dir)
    if not os.path.exists(path):
        os.makedirs(path)
    assert os.path.exists(path), "不存在%s目录" % root

print("change dir to", label_dirs[num])

if __name__ == "__main__":
    print("Start!")
    while True:
        return_value, image = camera.read()
        cv2.imshow("test", image)
        key = cv2.waitKey(1)
        if key >= ord('0') and key <=ord('9'):
            num = (key - 48) % length
            print("num = ",num)
            counter = START_NUM
            print("change dir to", label_dirs[num])
        elif key >= ord('a') and key <=ord('c'):
            num = (key - 48) % length + 9
            print("num = ",num)
            counter = START_NUM
            print("change dir to", label_dirs[num])
        elif key == ord('p'):
            path = "{}/{}/{}.png".format(root, label_dirs[num], counter)
            counter += 1
            name = "{}.png".format(counter)
            print(path)
            cv2.imwrite(path, image)
            time.sleep(0.2)
        elif key == ord('q'):
            print("exit!")
            break
camera.release()
cv2.destroyAllWindows()
