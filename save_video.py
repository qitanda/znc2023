# 调用摄像头采集视频，按esc退出，

import cv2

# 摄像头编号
cam = 0

print("loading camera ...")
camera = cv2.VideoCapture(cam)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # width
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  #height

fps=20         
size=(int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
      int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)))

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out=cv2.VideoWriter()
out.open("1.mp4", fourcc, fps, size)

print("save ...")
while True:
    ret, frame  = camera.read()
    if ret == False:
        print("camera error!")
        break
    # frame = cv2.flip(frame,1)
    out.write(frame)
    cv2.imshow("frame",frame)  
    key=cv2.waitKey(25)
    # print(1)
    if key==27:                  
        print("exit!")
        break
    success,frame=vc.read() 

camera.release() 
out.release() 
cv2.destroyAllWindows()
