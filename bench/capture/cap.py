# capture a serial stream
import serial, time, sys
from threading import Thread, Event


# params
# globals
logfile = "cap.out"
port = '/dev/ttyS6'
baudrate = 9600
timeout = 100 * (1.0 / baudrate)
go = Event()
eol = '\r'

def main():
    global go, ser
    try:
        go.set()
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        Thread(target=serThread).start()
        while 1:
            l = sys.stdin.readline()
            stamp = time.time()%1000
            print "%.3f>> " % stamp
            ser.write("%s%s" % (l[:-1],eol))
    except: stop()
    else: stop()

def serThread():
    global go, ser
    print "serThread starts"
    while go.is_set():
        if ser.in_waiting:
            l = ser.readline()
            stamp = time.time()%1000
            print "%.3f<< " % stamp
            sys.stdout.write("%r\n" % l)
    print "serThread stops"

def stop():
    global go, ser
    go.clear()
    del ser
