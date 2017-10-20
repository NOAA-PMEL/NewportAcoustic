# test char delays in response

from serial import Serial
from threading import Thread, Event
import time, sys

# globals set in init(), start()
# go = ser = None
# timer = 0

port = '/dev/ttyS2'
baudrate = 9600

def info():
    "globals which may be externally set"
    print "(go:%s)   " % (go.isSet())

def init():
    "set up globals"
    global ser, go, timer
    ser = Serial(port=port,baudrate=baudrate)
    timer = time.time()
    go = Event()

def start():
    "start reader thread"
    global go
    # threads run while go is set
    go.set()
    Thread(target=serThread).start()

def stop():
    global go
    "stop threads, close serial"
    if go: go.clear()

def serThread():
    "thread: loop looks for serial input; to stop set sergo=0"
    global go, ser, timer
    if not ser.is_open: ser.open()
    ser.buff = []
    try:
        while go.isSet():
            # CTD. syncMode, sample, settings
            while ser.in_waiting:
                c = ser.read()
                t = time.time()-timer
                ser.buff += [[c,t]]
                ser.write(c)
    except IOError, e:
        print "IOError on serial, calling stop() ..."
        stop()
    if ser.is_open: ser.close()

def stampPrint(buf):
    "print chars and timestamps"
    a = ser.buf
    if len(a) == 0:
        print "ser.buf is empty"
    for i in a:
        print "(%s %s)" % tuple(i)

init()
start()

while 1:
    con = sys.stdin.readline()
    if 'show' in con:
        stampPrint(ser.buf)
    elif 'exit' in con:
        break
    else :
        ser.buf = []
        ser.write(con)
        timer = time.time()

stop()
if ser.is_open: ser.close()
