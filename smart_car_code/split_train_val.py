# 数据集划分

# coding:utf-8

import os
import random
import argparse

parser = argparse.ArgumentParser()
# jpg文件的地址，根据自己的数据进行修改
parser.add_argument('--jpg_path', default='img_lab/images', type=str, help='input jpg label path')
# 数据集的划分，地址选择自己数据下的ImageSets/Main
parser.add_argument('--txt_path', default='img_lab/ImageSets/Main', type=str, help='output txt label path')

opt = parser.parse_args()

trainval_percent = 0.9  # 训练集和验证集在总数据集中占比
train_percent = 0.9  # 训练集在训练验证集中占比
jpgfilepath = opt.jpg_path
txtsavepath = opt.txt_path
total_jpg = os.listdir(jpgfilepath)
if not os.path.exists(txtsavepath):
    os.makedirs(txtsavepath)

num = len(total_jpg)
list_index = range(num)
tv = int(num * trainval_percent)
tr = int(tv * train_percent)
trainval = random.sample(list_index, tv)
train = random.sample(trainval, tr)

file_trainval = open(txtsavepath + '/trainval.txt', 'w')
file_test = open(txtsavepath + '/test.txt', 'w')
file_train = open(txtsavepath + '/train.txt', 'w')
file_val = open(txtsavepath + '/val.txt', 'w')
for i in list_index:
    name = total_jpg[i][:-4] + '\n'
    if i in trainval:
        file_trainval.write(name)
        if i in train:
            file_train.write(name)
        else:
            file_val.write(name)
    else:
        file_test.write(name)

file_trainval.close()
file_train.close()
file_val.close()
file_test.close()

sets = ['train', 'val', 'test']
# abs_path = '/home/dl501/demo/yolov5-master/mymydata'
abs_path = os.getcwd()
print(abs_path)

for image_set in sets:
    image_ids2 = open('img_lab/ImageSets/Main/%s.txt' % (image_set)).read().strip().split()
    list_file = open('img_lab/%s.txt' % (image_set), 'w')
    for image_id in image_ids2:
        list_file.write(abs_path + '/img_lab/images/%s.jpg ' % (image_id) + abs_path + '/annotations/%s.xml\n' % (image_id))
    list_file.close()
