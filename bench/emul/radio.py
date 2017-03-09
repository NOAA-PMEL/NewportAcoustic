# emulate radio / antenna module

from laraSer import Serial
from time import time, sleep
from shared import *
from threading import Thread, Event

# globals
go = ser = None

#
# these defaults may be changed by a dict passed to modGlobals
#
name = 'radio'
eol = '\r'
port = '/dev/ttyS8'
baudrate = 19200
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
    global go, ser
    while go.isSet():
        # iridium radio. TBD
        if ser.in_waiting:
            l = ser.getline()
            if '+PD' in l:
                ser.put(
                    ( '\r\n'
                    + 'UTC Date=%02d-%02d-%04d'
                    + ' Satellites Used=%2d'
                    + '\r\n'
                    ) % (1, 1, 2017, 11)
                )
            elif '+PT' in l:
                ser.put(
                    ( '\r\n'
                    + 'UTC Time=%02d:%02d:%02d.%03d'
                    + ' Satellites Used=%2d'
                    + '\r\n'
                    ) % (1, 1, 1, 1, 11)
                )
            elif '+PL' in l:
                ser.put(
                    ( '\r\n'
                    + 'Latitude=%02d:%02d.%04d%s'
                    + ' Longitude=%03d:%02d.%04d%s'
                    + ' Altitude=%.1f meters'
                    + ' Satellites Used=%2d'
                    + '\r\n'
                    ) % (1, 1, 1, 'N', 1, 1, 1, 'E', -0.1, 11)
                )
            elif '+CPAS' in l:
                ser.put(
                    ( '\r\n'
                    + '+CPAS:%03d'
                    + ' Satellites Used=%2d'
                    + '\r\n' )
                    % (1, 11)
                )
            elif l: pass


