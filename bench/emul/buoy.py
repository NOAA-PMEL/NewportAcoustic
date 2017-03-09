# emulate buoy

from laraSer import Serial
from time import time, sleep
from shared import *
from winch import depth
from threading import Thread, Event

# globals
go = ser = None

#
# these defaults may be changed by a dict passed to modGlobals
#
name = 'ctd'
eol = '\r\n'
port = '/dev/ttyS7'
baudrate = 9600
syncmode = 0

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

def open():
    "start serial and reader thread"
    global ser, go
    ser = Serial(port=port,baudrate=baudrate,name=name,eol=eol)
    # threads run while go is set
    go = Event()
    go.set()
    Thread(target=serThread).start()

def shut():
    "stop threads, close serial"
    go.clear()
    ser.close()


def serThread():
    "thread: loop looks for serial input; to stop set sergo=0"
    global go, ser, syncmode
    while go.isSet():
        # CTD. syncmode, sample, settings
        if ser.in_waiting:
            # syncmode is special, a trigger not a command, eol not required
            if syncmode:
                c = ser.get()
                if '\x00' in c:
                    # break
                    ser.log( "break ignored" )
                if '\x00' != c:
                    ctdOut()
                    # flush
                    ser.reset_input_buffer
            # command line. note: we don't do timeout
            else:
                # upper case is standard for commands, but optional
                l = ser.getline().upper()
                if 'TS' in l: 
                    ctdOut()
                elif 'SYNCMODE=Y' in l:
                    syncmode=1
                else: pass
                ser.put('S>')

def ctdOut():
    "instrument sample"
    # "# 20.6538,  0.01145,    0.217,   0.0622, 01 Aug 2016 12:16:50"
    # "\r\n# t.t,  c.c,  d.d,  s.s,  dd Mmm yyyy hh:mm:ss\r\n"

    # ctd delay to process, nominal 3.5 sec. Add variance?
    sleep(3.8)
    ###
    d="01 Aug 2016"
    t="12:16:50"
    # note: modify temp for ice
    ser.put("\r\n# %f, %f, %f, %f, %s %s\r\n" %
        (20.1, 0.01, depth(), 0.06, d, t))


