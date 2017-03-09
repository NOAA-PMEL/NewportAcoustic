# emulate LARA winch
from laraSer import Serial
from time import time, sleep
from shared import *
from threading import Thread, Event

# globals
motorOn = go = ser = None
buffOut = ''

#
# these defaults may be changed by a dict passed to modGlobals
#

# motor 0=off, 1=down, -1=up; motorLastTime=utc motor started
motorRunState = 0 
motorLastTime = 0.0

# cableStartLen is length of cable when motor started
cableLen = cableStartLen = 0
mooring = 30

# amodRate measured about 6.5 sec 
name = 'winch'
eol = '\r\n'
port = '/dev/ttyS9'
baudrate = 4800
amodDelay = 6.5

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
    global ser, go, motorOn
    ser = Serial(port=port,baudrate=baudrate,name=name,eol=eol)
    # threads run while go is set
    go = Event()
    go.set()
    Thread(target=serThread).start()
    motorOn = Event()
    motorOn.clear()
    Thread(target=motorThread).start()

def shut():
    "stop threads, close serial"
    go.clear()
    motorOn.set()
    ser.close()


def serThread():
    "thread: looks for serial input, output; sleeps to simulate amodDelay"
    while go.isSet():
        # acoustic modem. up, stop, down.
        if ser.in_waiting:
            amodInput()
        if buffOut:
            amodOutput()

def amodInput():
    "process input at serial, sleeps to simulate amodDelay"
    riseCmd = "#R,%s,03" % winchID
    riseRsp = "%%R,%s,00" % buoyID
    stopCmd = "#S,%s,00" % winchID
    stopRsp = "%%S,%s,00" % buoyID
    fallCmd = "#F,%s,00" % winchID
    fallRsp = "%%F,%s,00" % buoyID
    buoyAck = "%%S,%s,00" % winchID
    l = ser.getline()
    if len(l) > 6: sleep(amodDelay)
    # rise
    if riseCmd in l:
        motor(-1)
        ser.log( "up at depth %s" % depth() )
        sleep(amodDelay)
        ser.putline(riseRsp)
    # stop
    elif stopCmd in l:
        motor(0)
        ser.log( "stop at depth %s" % depth() )
        sleep(amodDelay)
        ser.putline(stopRsp)
    # fall
    elif fallCmd in l:
        motor(1)
        ser.log( "down at depth %s" % depth() )
        sleep(amodDelay)
        ser.putline(fallRsp)
    # buoy responds to stop after dock or slack
    elif buoyAck in l:
        ser.log( "buoy response %s" % l )
    # something strange
    elif l:
        ser.log("amod: unexpected %r" % l)

def amodOutput():
    "Sleep to emulate amodDelay, put buffOut"
    global buffOut
    # check
    if not buffOut: return ser.log("err: amodOutput(): empty buffOut")
    # one line out
    sleep(amodDelay)
    b = buffOut
    # end of line
    e = b.find(ser.eol)
    if e>0: 
        # found eol, include
        e += len(ser.eol)
    else: e = len(b)
    ser.put( b[:e] )
    buffOut = b[e:]

def amodPut(s):
    "Buffer output for slow sending by amodOutput"
    global buffOut
    buffOut += s
    

def motor(state):
    "set motorRunState to 0=off, 1=down, -1=up; motorOn event"
    global motorRunState, motorLastTime, motorOn
    if motorRunState==state: 
        return ser.log( "motor state already is %d" % state )
    #
    motorLastTime = time()
    motorRunState = state
    ser.log( "motor state set to %s" % state )
    # if turning on, notify motorThread
    if motorRunState: motorOn.set()
    else: motorOn.clear()

def motorThread():
    "thread: if motor is on, update cableLen, check dock and slack"
    global cableLen, motorLastTime, motorRunState, motorOn
    while go.isSet():
        motorOn.wait()
        # up
        t = time()
        if motorRunState==-1:
            # simple linear
            cableLen += (t - motorLastTime) * .331
            if slack():
                motor(0)
                # no amod delay here, sleep(amodDelay) is in serThread
                amodPut("#S,%s,00%s" % (buoyID, ser.eol))
                ser.log( "surfaced, sending stop command to buoy" )
        # down
        if motorRunState==1:
            # simple linear
            cableLen -= (t - motorLastTime) * .2
            if docked():
                motor(0)
                # no amod delay here, sleep(amodDelay) is in serThread
                amodPut("#S,%s,00%s" % (buoyID, ser.eol))
                ser.log( "docked, sending stop command to buoy" )
        motorLastTime = t

def slack():
    "determine if the cable is slack"
    global mooring, cableLen
    # TBD
    return depth()<.1

def docked():
    "are we docked?"
    global cableLen
    if cableLen<=0:
        cableLen=0
        return 1

def depth():
    "mooring - cableLen, mod by current"
    global mooring, cableLen
    # TBD
    return mooring-cableLen
