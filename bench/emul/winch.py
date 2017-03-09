# emulate LARA winch
from laraSer import Serial
from time import time, sleep
from shared import *
from threading import Thread, Event

# globals
go = ser = None

#
# these defaults may be changed by a dict passed to init
#

# motor 0=off, 1=down, -1=up; motorStartTime=utc motor started
motorRunState = 0 
motorStartTime = 0.0

# cableStartLen is length of cable when motor started
cableLen = cableStartLen = 0
mooring = 30

# amodRate measured about 6.5 sec for 8 chars + \r\n
amodRate = 6.5/10
port = '/dev/ttyS9'
baudrate = 4800
name = 'amod'

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
    while go.isSet():
        # acoustic modem. up, stop, down.
        if ser.in_waiting:
            l = ser.getline('\r\n')
            sleep(amodRate * (len(l)+2))
            # up command
            if ("#R,%s,03" % winchID) in l:
                motor(-1)
                ser.log( "phase=2, up" )
                amodSend("%%R,%s,00" % buoyID)
            # stop command
            elif ("#S,%s,00" % winchID) in l:
                motor(0)
                ser.log( "phase=3, surface" )
                amodSend("%%S,%s,00" % buoyID)
            # down command
            elif ("#F,%s,00" % winchID) in l:
                motor(1)
                ser.log( "phase=4, down" )
                amodSend("%%F,%s,00" % buoyID)
            # buoy responds to stop from dock or slack
            elif ("%%S,%s,00" % buoyID) in l:
                ser.log( "buoy response %s" % l )
            # something strange
            elif l:
                ser.log("amod: unexpected %r" % l)

def motor(m):
    "set motorRunState to 0=off, 1=down, -1=up"
    global motorRunState, motorStartTime, cableStartLen
    if motorRunState==m: 
        ser.log( "winch.motor(): already is %d" % m )
        return -1

    # cable update, cableStartLen, motorRunState, motorStartTime
    cableStartLen = cable()
    motorRunState = m
    motorStartTime = time()


def slack():
    "determine if the cable is slack"
    # TBD
    return 0
    if r:
        amodSend("#S,%s,00" % buoyID)
    return r


def cable():
    "update cable position, return length"
    global cableLen, cableStartLen, motorRunState
    # up
    if motorRunState==-1:
        # simple linear
        cableLen = cableStartLen + (time() - motorStartTime) * .331
        if depth()<0.1:
            #surfaced
            motorRunState = 0
            ser.log( "phase=3, surfaced, sending stop command" )
            amodSend("#S,%s,00" % buoyID)
            
    # down
    if motorRunState==1:
        # simple linear
        cableLen = cableStartLen - (time() - motorStartTime) * .2
        if cableLen<0:
            # docked
            cableLen = 0
            motorRunState = 0
            ser.log( "phase=1, docked, sending stop command" )
            amodSend("#S,%s,00" % buoyID)
    return cableLen

def depth():
    "mooring - cableLen, mod by current"
    return mooring-cableLen

def amodSing():
    "call this function several times per second"
    # depricated: use this without threading
    global amodSong, amodNext, ser
    if not amodSong: return false
    now = time()
    r = 0
    # if several seconds have passed, several 'notes' (should not happen)
    while amodNext<now:
        # note: ser set global by winch.init()
        ser.put(amodSong[:1])
        amodSong = amodSong[1:]
        # any more notes left?
        if amodSong:
            amodNext += amodRate
        r += 1
    return r
            

def amodSend(amodSong):
    "acoustic modem sings a slow 'amodSong', start singing the string"
    # simplified by threading
    for note in amodSong:
        ser.put(note)
        sleep(amodRate)

