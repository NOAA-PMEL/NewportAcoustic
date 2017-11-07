# test char delays in response

from serial import Serial
from serial.tools.list_ports import comports
from threading import Thread, Event
import time, sys

args = len(sys.argv)
if args>1 and 'h' in sys.argv[1]:
    print "usage: %s port" % sys.argv[0]
    sys.exit(-1)
#
if args>1:  port = '/dev/ttyS%s' % sys.argv[1]
else:       port = '/dev/ttyS3'
if args>2:  baudrate = int(sys.argv[2])
else:       baudrate = 9600
echo = False
devEol = '\r'
outputL = 70

def init():
    "set up globals"
    global buf, ser, go, timer
    buf = []
    go = Event()
    timer = time.time()
    global port, baudrate
    try:
        ser = Serial(port=port,baudrate=baudrate)
    except:
        print "no serial on %s" % port
        print sys.exc_info()[1]
        for i in comports(): print i[0]
        ser = None
        sys.exit()

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
            t = time.time()
            buf += [[c,t]]
            sys.stdout.write(c)
            sys.stdout.flush()
            if echo:
                ser.write(c)

def stampPrint(buf):
    "print chars and timestamps"
    line = ''
    tLast = tLong = diff = 0.0
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
        out += " %.3f) " % (t-timer)
        if tLast>timer: 
            diff = t-tLast
            if diff>tLong:
                tLong = diff
        tLast = t
        # add to line
        outL = len(out)
        lineL = len(line)
        if lineL+outL>outputL:
            sys.stdout.write(line + '\n')
            line = ''
        line += out
    sys.stdout.write(line + '\n')
    sys.stdout.write("Longest = %.3f\n" % tLong)
    sys.stdout.flush()

#

init()
start()

while 1:
    # take \n off end of readline
    con = sys.stdin.readline()[0:-1]
    if 'show' in con:
        stampPrint(buf)
        buf = []
    elif 'exit' in con:
        break
    else :
        timer = time.time()
        ser.write(con + devEol)

stop()
if ser.is_open: ser.close()
