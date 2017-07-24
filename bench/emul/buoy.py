# emulate buoy

#from laraSer import Serial
from laraSer import Serial
from shared import *
from winch import depth
from threading import Thread, Event
from time import sleep
import time

# globals set in init(), start()
# go = ser = None
# sleepMode = syncMode = False
# timeOff = 0

name = 'ctd'
eol = '\r'        # input is \r, output \r\n
port = '/dev/ttyS5'
baudrate = 9600
CTD_DELAY = 1

def info():
    "globals which may be externally set"
    print "(go:%s)   syncMode=%s   sleepMode=%s" % \
        (go.isSet(), sleepMode, syncMode)

def init():
    "set globals to defaults"
    global ser, go, sleepMode, syncMode
    ser = None
    ser = Serial(port=port,baudrate=baudrate,name=name,eol=eol)
    sleepMode = syncMode = False
    timeoff = 0
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
    global go, ser, syncMode, sleepMode
    if not ser.is_open: ser.open()
    ser.buff = ''
    try:
        while go.isSet():
            # CTD. syncMode, sample, settings
            if ser.in_waiting:
                # syncMode is special, a trigger not a command, eol not required
                # consume input while pondering
                if syncMode and sleepMode:
                    c = ser.get()
                    if '\x00' in c:
                        # break
                        ser.log( "break; syncMode off, flushing %r" % ser.buff )
                        syncMode = False
                        sleepMode = False
                        ser.buff = ''
                        ser.reset_input_buffer
                    else:
                        ctdOut()
                else: # not sync & sleep
                    # command line. note: we don't do timeout
                    # upper case is standard for commands, but optional
                    l = ser.getline(echo=1).upper()
                    if l:
                        l = l[:-len(ser.eol)]
                        if 'TS' in l: 
                            ctdOut()
                        elif 'DATE' in l:
                            dt = l[l.find('=')+1:]
                            setDateTime(dt)
                            ser.log( "set date time %s -> %s" % \
                                (dt, ctdDateTime()))
                        elif 'SYNCMODE=Y' in l:
                            syncMode = True
                            ser.log( "syncMode pending (when ctd sleeps)")
                        elif 'QS' in l:
                            sleepMode = True
                            ser.log("ctd sleepMode")
                        if sleepMode != True: 
                            ser.put('S>')
        # while go:
    except IOError, e:
        print "IOError on serial, calling buoy.stop() ..."
        stop()
    if ser.is_open: ser.close()

def setDateTime(dt):
    "set ctdClock global timeOff from command in seabird format"
    global timeOff
    # datetime=mmddyyyyhhmmss to python time struct
    pyTime = time.strptime(dt,"%m%d%Y%H%M%S")
    # python time struct to UTC
    utc = time.mktime(pyTime)
    # offset between emulated ctd and this PC clock
    timeOff = time.time()-utc

def ctdDateTime():
    "use global timeOff set by setDateTime() to make a date"
    global timeOff
    f='%d %b %Y %H:%M:%S'
    return time.strftime(f,time.localtime(time.time()-timeOff))

def ctdDelay():
    "Delay for response. TBD, variance"
    global CTD_DELAY
    return CTD_DELAY

# Temp, conductivity, depth, fluromtr, PAR, salinity, time
# 16.7301,  0.00832,    0.243, 0.0098, 0.0106,   0.0495, 14 May 2017 23:18:20
def ctdOut():
    "instrument sample"
    # "\r\n# t.t, c.c, d.d, f.f, p.p, s.s,  dd Mmm yyyy hh:mm:ss\r\n"

    # ctd delay to process, nominal 3.5 sec. Add variance?
    sleep(ctdDelay())
    ###
    # note: modify temp for ice
    ser.put("\r\n# %f, %f, %f, %f, %f, %f, %s\r\n" %
        (20.1, 0.01, depth(), 0.01, 0.01, 0.06, ctdDateTime() ))

#def modGlobals(**kwargs):
#    "change defaults from command line"
#    # change any of module globals, most likely mooring or cableLen
#    if kwargs:
#        # update module globals
#        glob = globals()
#        logmsg = "params: "
#        for (i, j) in kwargs.iteritems():
#            glob[i] = j
#            logmsg += "%s=%s " % (i, j)

init()
