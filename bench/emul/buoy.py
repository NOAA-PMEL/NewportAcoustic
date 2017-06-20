# emulate buoy

#from laraSer import Serial
from laraSer import Serial
from shared import *
from winch import depth
from threading import Thread, Event
from time import sleep
import time

# globals
go = ser = None

#
# these defaults may be changed by a dict passed to modGlobals
#
name = 'ctd'
eol = '\r\n'
port = '/dev/ttyS7'
baudrate = 9600
sleepMode = syncMode = 0

def modGlobals(**kwargs):
    "change defaults from command line"
    # change any of module globals, most likely mooring or cableLen
    if kwargs:
        # update module globals
        glob = globals()
        logmsg = "params: "
        for (i, j) in kwargs.iteritems():
            glob[i] = j
            logmsg += "%s=%s " % (i, j)

def run():
    "start serial and reader thread"
    global ser, go
    ser = Serial(port=port,baudrate=baudrate,name=name,eol=eol)
    # threads run while go is set
    go = Event()
    go.set()
    Thread(target=serThread).start()

def stop():
    "stop threads, close serial"
    if go: go.clear()
    if ser: ser.close()


def serThread():
    "thread: loop looks for serial input; to stop set sergo=0"
    global go, ser, syncMode, sleepMode
    while go.isSet():
        # CTD. syncMode, sample, settings
        if ser.in_waiting:
            # syncMode is special, a trigger not a command, eol not required
            if syncMode and sleepMode:
                c = ser.get()
                if '\x00' in c:
                    # break
                    ser.log( "break; syncMode off, flushing %r" % ser.buff )
                    syncMode = 0
                    sleepMode = 0
                    ser.buff = ''
                    ser.reset_input_buffer
                else:
                    ctdOut()
            # command line. note: we don't do timeout
            else:
                # upper case is standard for commands, but optional
                l = ser.getline().upper()
                if 'TS' in l: 
                    ctdOut()
                elif 'DATE' in l:
                    dt = l[l.find('=')+1:]
                    setDateTime(dt)
                    ser.log( "set date time %s -> %s" % (dt, ctdDateTime()))
                elif 'SYNCMODE=Y' in l:
                    syncMode=1
                    ser.log( "syncMode pending (when ctd sleeps)")
                elif 'QS' in l:
                    sleepMode = 1
                    ser.log("ctd sleepMode")
                if sleepmode != 1: 
                    ser.put('S>')

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

def ctdOut():
    "instrument sample"
    # CTD with fluro, par
    # Temp, conductivity, depth, fluromtr, PAR, salinity, time
    # 16.7301,  0.00832,    0.243, 0.0098, 0.0106,   0.0495, 14 May 2017 23:18:20
    # "\r\n# t.t, c.c, d.d, f.f, p.p, s.s,  dd Mmm yyyy hh:mm:ss\r\n"

    # ctd delay to process, nominal 3.5 sec. Add variance?
    sleep(3.8)
    ###
    # note: modify temp for ice
    ser.put("\r\n# %f, %f, %f, %f, %f, %f, %s\r\n" %
        (20.1, 0.01, depth(), 0.01, 0.01, 0.06, ctdDateTime() ))


