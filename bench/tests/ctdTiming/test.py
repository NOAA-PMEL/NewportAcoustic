# test char delays in response

from serial import Serial
from threading import Thread, Event
import time, sys

# globals set in init(), start()
# go = ser = None
# timer = 0

port = '/dev/ttyS2'
baudrate = 9600
#echo = True
echo = False
devEol = '\r\n'
outputL = 70

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
            sys.stdout.flush()
            if echo:
                ser.write(c)

def stampPrint(buf):
    "print chars and timestamps"
    line = ''
    # 
    if len(buf) == 0:
        print "buf is empty"
        return
    for (c,t) in buf:
        out = "("
        d = ord(c)
        if c == '\n':
            out += "\\n"
        elif c == '\r':
            out += "\\r"
        elif c == ' ':
            out += '_'
        elif d in range(32,127):
            out += "%s" % c
        else:
            out += "%02X" % d
        out += " %.3f) " % t
        # add to line
        outL = len(out)
        lineL = len(line)
        if lineL+outL>outputL
            sys.stdout.write(line + '\n')
            sys.stdout.flush()
            line = ''
        line += out
    sys.stdout.write(line + '\n')
    sys.stdout.flush()

#

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
        ser.write(con + devEol)

stop()
if ser.is_open: ser.close()
