import os #导入模块
filename = 'D:\\智慧交通\\smart_car_code\\images\\swordfish'
list_path = os.listdir(filename)  #读取文件夹里面的名字
id = 73
for index in list_path:  #list_path返回的是一个列表   通过for循环遍历提取元素
    name = index.split('.')[0]   #split字符串分割的方法 , 分割之后是返回的列表 索引取第一个元素[0]
    kid = index.split('.')[-1]   #[-1] 取最后一个
    path = filename + '\\' + index
    if id < 10:
        new_path = filename + '\\' + '0700' + str(id) + '.' + kid
    elif id < 100:
        new_path = filename + '\\' + '070' + str(id) + '.' + kid
    else:
        new_path = filename + '\\' + '07' + str(id) + '.' + kid
    # new_path = filename + '\\' + str(id) + '.' + kid
    id += 1
    os.rename(path, new_path) #重新命名

print('修改完成')