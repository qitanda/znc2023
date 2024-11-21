import threading
def func1(a):
    #Do something
    a+=1
    print('当前线程数为{}'.format(threading.active_count()))
    # if a>5:
    #     return
    t=threading.Timer(5,func1,(a,))
    t.start()
func1(0)