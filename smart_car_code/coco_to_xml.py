import os
import numpy as np
import codecs
import json
from glob import glob
import cv2
import shutil
# from sklearn.model_selection import train_test_split

# 1.存放的json标签路径
labelme_path = "annotations/dataset/labels/"

# 原始labelme标注数据路径
saved_path = "annotations/dataset/VOC2007/"
# 保存路径
isUseTest = True  # 是否创建test集

# 2.创建要求文件夹
if not os.path.exists(saved_path + "Annotations"):
    os.makedirs(saved_path + "Annotations")
if not os.path.exists(saved_path + "JPEGImages/"):
    os.makedirs(saved_path + "JPEGImages/")
if not os.path.exists(saved_path + "ImageSets/Main/"):
    os.makedirs(saved_path + "ImageSets/Main/")

# 3.获取待处理文件
files = glob(labelme_path + "*.json")
files = [i.replace("\\", "/").split("/")[-1].split(".json")[0] for i in files]
print(files)

# 4.读取标注信息并写入 xml
for json_file_ in files:
    json_filename = labelme_path + json_file_ + ".json"
    json_file = json.load(open(json_filename, "r", encoding="utf-8"))
    # height, width, channels = cv2.imread('D:\\智慧交通\\smart_car_code\\smart_car_code\\test\\' + '1' + '.jpg')\
    #     .shape
    height = 480
    width = 640
    channels = 3
    with codecs.open(saved_path + "Annotations/" + json_file_ + ".xml", "w", "utf-8") as xml:

        xml.write('<annotation>\n')
        xml.write('\t<folder>' + 'CELL_data' + '</folder>\n')
        xml.write('\t<filename>' + json_file_ + ".jpg" + '</filename>\n')
        xml.write('\t<source>\n')
        xml.write('\t\t<database>CELL Data</database>\n')
        xml.write('\t\t<annotation>CELL</annotation>\n')
        xml.write('\t\t<image>bloodcell</image>\n')
        xml.write('\t\t<flickrid>NULL</flickrid>\n')
        xml.write('\t</source>\n')
        xml.write('\t<owner>\n')
        xml.write('\t\t<flickrid>NULL</flickrid>\n')
        xml.write('\t\t<name>CELL</name>\n')
        xml.write('\t</owner>\n')
        xml.write('\t<size>\n')
        xml.write('\t\t<width>' + str(width) + '</width>\n')
        xml.write('\t\t<height>' + str(height) + '</height>\n')
        xml.write('\t\t<depth>' + str(channels) + '</depth>\n')
        xml.write('\t</size>\n')
        xml.write('\t\t<segmented>0</segmented>\n')# 是否用于分割（在图像物体识别中01无所谓）
        cName = json_file["categories"]
        Name = cName[0]["name"]
        print(Name)
        for multi in json_file["annotations"]:
            points = np.array(multi["bbox"])
            labelName = Name
            xmin = points[0]
            xmax = points[0]+points[2]
            ymin = points[1]
            ymax = points[1]+points[3]
            label = Name
            if xmax <= xmin:
                pass
            elif ymax <= ymin:
                pass
            else:
                xml.write('\t<object>\n')
                xml.write('\t\t<name>' + labelName + '</name>\n')# 物体类别
                xml.write('\t\t<pose>Unspecified</pose>\n')# 拍摄角度
                xml.write('\t\t<truncated>0</truncated>\n')# 是否被截断（0表示完整）
                xml.write('\t\t<difficult>0</difficult>\n')# 目标是否难以识别（0表示容易识别）
                xml.write('\t\t<bndbox>\n')
                xml.write('\t\t\t<xmin>' + str(int(xmin)) + '</xmin>\n')
                xml.write('\t\t\t<ymin>' + str(int(ymin)) + '</ymin>\n')
                xml.write('\t\t\t<xmax>' + str(int(xmax)) + '</xmax>\n')
                xml.write('\t\t\t<ymax>' + str(int(ymax)) + '</ymax>\n')
                xml.write('\t\t</bndbox>\n')
                xml.write('\t</object>\n')
                print(json_filename, xmin, ymin, xmax, ymax, label)
        xml.write('</annotation>')

# 5.复制图片到 VOC2007/JPEGImages/下
image_files = glob("annotations/dataset/images/" + "*.jpg")
print("copy image files to VOC007/JPEGImages/")
for image in image_files:
    shutil.copy(image, saved_path + "JPEGImages/")


