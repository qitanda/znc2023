import cv2
import json
from tqdm import tqdm
import xml.etree.ElementTree as ET

def pretty_xml(element, indent="\t", newline="\n", level=0):  # elemnt为传进来的Elment类，参数indent用于缩进，newline用于换行
    if element:  # 判断element是否有子元素
        if (element.text is None) or element.text.isspace():  # 如果element的text没有内容
            element.text = newline + indent * (level + 1)
        else:
            element.text = newline + indent * (level + 1) + element.text.strip() + newline + indent * (level + 1)
            # else:  # 此处两行如果把注释去掉，Element的text也会另起一行
            # element.text = newline + indent * (level + 1) + element.text.strip() + newline + indent * level
    temp = list(element)  # 将element转成list
    for subelement in temp:
        if temp.index(subelement) < (len(temp) - 1):  # 如果不是list的最后一个元素，说明下一个行是同级别元素的起始，缩进应一致
            subelement.tail = newline + indent * (level + 1)
        else:  # 如果是list的最后一个元素， 说明下一行是母元素的结束，缩进应该少一个
            subelement.tail = newline + indent * level
        pretty_xml(subelement, indent, newline, level=level + 1)  # 对子元素进行递归操作

def helper(infos,shape=None,save=None):
    root=ET.Element("annotation")
    folder=ET.SubElement(root,"folder")
    folder.text="COCO2017"
    img_name=list(infos.keys())[0].strip()
    file_name=ET.SubElement(root,"filename")
    file_name.text=img_name
    if shape:
        h,w,c=shape[0],shape[1],shape[2]
    else:
        img=cv2.imread(img_name)
        h,w,c=img.shape
    size=ET.SubElement(root,"size")
    width=ET.SubElement(size,"width")
    width.text=str(w)
    height=ET.SubElement(size,"height")
    height.text=str(h)
    depth=ET.SubElement(size,"depth")
    depth.text=str(c)
    segmented=ET.SubElement(root,"segmented")
    segmented.text="0"
    for key in infos.keys():
        bboxes=infos[key]
        for bbox in bboxes:
            object_tag=ET.SubElement(root,"object")
            name=ET.SubElement(object_tag,"name")
            name.text=bbox[-1]
            pose=ET.SubElement(object_tag,"pose")
            pose.text="Unspecified"
            truncated=ET.SubElement(object_tag,"truncated")
            truncated.text="0"
            diff=ET.SubElement(object_tag,"difficult")
            diff.text="0"
            bndbox=ET.SubElement(object_tag,"bndbox")
            bbox[0]=str(int(bbox[0]))
            bbox[1]=str(int(bbox[1]))
            bbox[2]=str(int(bbox[2]))
            bbox[3]=str(int(bbox[3]))
            xmin=ET.SubElement(bndbox,"xmin")
            xmin.text=bbox[0]
            ymin=ET.SubElement(bndbox,"ymin")
            ymin.text=bbox[1]
            xmax=ET.SubElement(bndbox,"xmax")
            xmax.text=bbox[2]
            ymax=ET.SubElement(bndbox,"ymax")
            ymax.text=bbox[3]
    pretty_xml(root)
    tree=ET.ElementTree(root)
    xml_name=img_name.replace("jpg","xml")
    tree.write(save+"\\"+xml_name,encoding="utf-8",xml_declaration=False)


if __name__=="__main__":
    path1="D:\\智慧交通\\smart_car_code\\smart_car_code\\annotations\\labels_my-project-name_2023-07-04-09-49-44.json"
    # path2="F:\\Mrfang\\COCO\\COCO2017\\annotations\\instances_val2017.json"
    save1="D:\\智慧交通\\smart_car_code\\smart_car_code\\annotations\\voc_xml"
    # save2="F:\\Mrfang\\COCO\\COCO2017\\annotations\\val_xml"
    path_txt="D:\\智慧交通\\smart_car_code\\smart_car_code\\annotations\\img_no_label.txt"

    with open(path1,"r") as f:
        json_dict=json.loads(f.read())
    cate=json_dict["categories"]
    cls_name=[0]*91
    for cate_dict in cate:
        cls_name[cate_dict["id"]]=cate_dict["name"]

    image_info=json_dict["images"]
    image_nums=len(image_info)
    print(image_nums)
    img_id=[]
    for index in tqdm(range(image_nums),desc="get_img_id",ncols=80,unit="img"):
        img_id.append(image_info[index]["id"])
    id_min=min(img_id)
    id_max=max(img_id)
    file_list=[-1]*(id_max+1)
    for index in tqdm(range(image_nums),desc="get_img_info",ncols=80,unit="img"):
        file_name=image_info[index]["file_name"]
        width=image_info[index]["width"]
        height=image_info[index]["height"]
        t=image_info[index]["id"]
        file_list[t]=[file_name,width,height]

    annot_list=[[] for _ in range(id_max+1)]
    annot=json_dict["annotations"]
    for index in tqdm(range(len(annot)),desc="get_object_info",ncols=80,unit="object"):
        annot[index]["bbox"][2]=annot[index]["bbox"][2]+annot[index]["bbox"][0]
        annot[index]["bbox"][3]=annot[index]["bbox"][3]+annot[index]["bbox"][1]
        annot[index]["bbox"].append(cls_name[annot[index]["category_id"]])
        annot_list[annot[index]["image_id"]].append(annot[index]["bbox"])

    out_file=open(path_txt,"w")
    for index in tqdm(range(id_min,id_max+1),desc="get_xml",ncols=80,unit="xml"):
        if annot_list[index]==[] and file_list[index]!=-1:
            out_file.write(file_list[index][0]+"\n")
            continue
        if file_list[index]==-1:
            continue
        box=annot_list[index]
        t1={file_list[index][0]:box}
        t2=[str(file_list[index][2]),str(file_list[index][1]),"3"]
        helper(t1,t2,save1)