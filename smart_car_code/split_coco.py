from __future__ import print_function
import json
json_file='D:\\智慧交通\\smart_car_code\\smart_car_code\\annotations\\labels_my-project-name_2023-07-04-09-49-44.json'
# Object Instance 类型的标注
# person_keypoints_val2017.json
# Object Keypoint 类型的标注格式
# captions_val2017.json
# Image Caption的标注格式
data=json.load(open(json_file,'r'))
data_2={}
# da ta_2['info']=data['info']
# data_2['licenses']=data['licenses']
for i in range(430): # 一共234张图片
    data_2['images']=[data['images'][i]] # 只提取第i张图片
    data_2['categories']=data['categories']
    annotation=[] # 通过imgID 找到其所有对象
    imgID=data_2['images'][0]['id']
    for ann in data['annotations']:
        if ann['image_id']==imgID:
            annotation.append(ann)
    data_2['annotations']=annotation # 保存到新的JSON文件，便于查看数据特点
    savepath = 'D:\\智慧交通\\smart_car_code\\smart_car_code\\annotations\\coco\\' + str(imgID).zfill(6) + '.json'
    json.dump(data_2,open(savepath,'w'),indent=4) # indent=4 更加美观显示

