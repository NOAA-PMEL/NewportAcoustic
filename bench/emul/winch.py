# emulate winch v3
from laraSer import Serial
from serial.tools.list_ports import comports
from threading import Thread, Event
from time import time, sleep
from design import *
import floats

# globals set in init(), start()

# amodRate measured about 6.5 sec 
name = 'winch'
baudrate = 4800
eol = '\n'
# select port 0-n of multiport serial
portSelect = 3

amodDelay = 5.5

def info():
    global go, cableLen, motorRunState
    "globals which may be externally set"
    print "(go:%s)   motor('%s')   cableLen=%.2f" % \
        (go.isSet(), motorRunState, cableLen)

def init(portSel=portSelect):
    "set global vars to defaults"
    global go, ser, cableLen, motorRunState, motorOn
    cableLen = 0
    # motorRunState off, down, up
    motorRunState = 'off' 
    motorOn = Event()
    go = Event()
    try:
        # select port 0-n of multiport serial
        port = comports()[portSel].device
        ser = Serial(port=port,baudrate=baudrate,name=name)
    except:
        print "no serial for %s" % name
        ser = None

def start():
    "start serial and reader thread"
    global go, ser, buffOut
    buffOut = ''
    # threads run while go is set
    go.set()
    # during test, we may be running with no serial
    Thread(target=serThread).start()
    Thread(target=motorThread).start()

def stop():
    global go, motorOn
    "stop threads, close serial"
    if go: go.clear()
    if not motorOn.isSet(): 
        # release motor thread, if motor is off
        motorOn.set()
        motorOn.clear()

def serThread():
    "thread: looks for serial input, output; sleeps to simulate amodDelay"
    global ser, go
    if not ser.is_open: ser.open()
    try:
        while go.isSet():
            # acoustic modem. up, stop, down.
            if ser.in_waiting:
                amodInput()
            if buffOut:
                amodOutput()
        # while go
    except IOError, e:
        print "IOError on serial, calling stop() ..."
        stop()
    if ser.is_open: ser.close()


def amodInput():
    "process input at serial, sleeps to simulate amodDelay"
    # #R,01,03
    riseCmd = "#R,%s,03" % winchID
    riseRsp = "%%R,%s,00" % buoyID
    stopCmd = "#S,%s,00" % winchID
    stopRsp = "%%S,%s,00" % buoyID
    fallCmd = "#F,%s,00" % winchID
    fallRsp = "%%F,%s,00" % buoyID
    buoyAck = "%%S,%s,00" % winchID
    l = ser.getline()
    if not l: return
    ser.log( "hearing %s" % l )
    if len(l) > 6: sleep(amodDelay)
    # rise
    if riseCmd in l:
        motor('up')
        sleep(amodDelay)
        ser.putline(riseRsp)
    # stop
    elif stopCmd in l:
        motor('off')
        sleep(amodDelay)
        ser.putline(stopRsp)
    # fall
    elif fallCmd in l:
        motor('down')
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
    # note - should use threading.Timer for this
    buffOut += s

def motor(state):
    "set motorRunState to off, down, up; motorOn event"
    global ser, motorRunState, motorOn
    if state not in ( 'off', 'down', 'up'):
        return ser.log( "motor(up|off|down), not '%s'" % state )
    #
    motorRunState = state
    ser.log( "motor %s with cableLen at %.2f" % (state,cableLen) )
    if motorRunState=='off': motorOn.clear()
    else: motorOn.set()

def motorThread():
    "when motor is on: update cableLen, check dock and slack"
    global ser, go, cableLen, motorRunState, motorOn
    # motor could be on when emulation starts
    while go.isSet(): 
        motorOn.wait()
        motorLastTime = time()
        sleep(.1)
        # up
        if motorRunState=='up':
            cableLen += (time() - motorLastTime) * .331
            if slack():
                ser.log( "surfaced" )
                motor('off')
                amodPut("#S,%s,00%s" % (buoyID, ser.eol))
        # down
        if motorRunState=='down':
            cableLen -= (time() - motorLastTime) * .2
            if docked():
                ser.log( "docked" )
                cableLen=0
                motor('off')
                amodPut("#S,%s,00%s" % (buoyID, ser.eol))

def slack():
    "determine if the cableLen is slack"
    return floats.depth()<.1

def docked():
    "are we docked?"
    global cableLen
    return cableLen<.1

def cable(new=None):
    "set or get cableLen"
    global cableLen
    if new is not None:
        cableLen = new
    return cableLen

init()
