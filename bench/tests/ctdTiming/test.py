# test char delays in response

from serial import Serial
from threading import Thread, Event
import time, sys

# globals set in init(), start()
# go = ser = None
# timer = 0

port = '/dev/ttyS2'
baudrate = 9600
echo = True

def info():
    "globals which may be externally set"
    print "(go:%s)   " % (go.isSet())

def init():
    "set up globals"
    global buf, ser, go, timer
    buf = []
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
    global buf, go, ser, timer, echo
    if not ser.is_open: ser.open()
    buf = []
    while go.isSet():
        # CTD. syncMode, sample, settings
        if ser.in_waiting:
            c = ser.read()
            t = time.time()-timer
            buf += [[c,t]]
            sys.stdout.write(c)
            if echo:
                ser.write(c)

def stampPrint(buf):
    "print chars and timestamps"
    a = buf
    if len(a) == 0:
        print "buf is empty"
    for (c,t) in a:
        print "(",
        d = ord(c)
        if c == '\n':
            print "\\n",
        elif c == '\r':
            print "\\r",
        elif d in range(32,127):
            print "%s" % c,
        else:
            print "%02X" % d,
        print "%.3f )" % t

init()
start()

while 1:
    con = sys.stdin.readline()
    if 'show' in con:
        stampPrint(buf)
    elif 'exit' in con:
        break
    else :
        buf = []
        timer = time.time()
        ser.write(con)

stop()
if ser.is_open: ser.close()
