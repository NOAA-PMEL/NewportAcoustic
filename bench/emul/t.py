from time import sleep
from threading import Semaphore, Thread

s=Semaphore()
quit=1
def stop():
    quit=0
def run():
    quit=1
    while quit:
        sleep(3)
        s.acquire()
        print 5
        s.release()
t=Thread(target=run)
s.acquire()
t.start()
