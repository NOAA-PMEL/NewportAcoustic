# emulate buoy

from laraSer import Serial
from time import time, sleep
from shared import *
from winch import depth
from threading import Thread, Event

# globals
go = ser = None

#
# these defaults may be changed by a dict passed to init
#
# amodRate measured about 6.5 sec for 8 chars
name = 'ctd'
port = '/dev/ttyS7'
baudrate = 9600
syncmode = 0
# eol = '\r\n'

def turnOn(**kwargs):
    "reset defaults from kwargs, start serial and reader thread"
    # change any of defaults, most likely mooring, cableLen
    global ser, go
    if kwargs:
        # update module globals
        glob = globals()
        logmsg = "params: "
        for (i, j) in kwargs.iteritems():
            glob[i] = j
            logmsg += "%s=%s " % (i, j)

    ser = Serial(port=port,baudrate=baudrate)
    ser.name = name
    if kwargs: ser.log( logmsg )
    # thread to watch serial
    go = Event()
    go.set()
    thread = Thread(target=run)
    thread.start()

def turnOff():
    "stops thread, unset loop condition"
    go.clear()

def run():
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


