# emulate LARA winch
from laraSer import Serial
from time import time, sleep
from shared import *
from threading import Thread, Event

# globals set in init(), start()
# motorOn = go = ser = None
# buffOut = ''
#motorRunState = off, down, up
#cable = 0
#mooring = 20

# amodRate measured about 6.5 sec 
name = 'winch'
eol = '\n'
port = '/dev/ttyS6'
baudrate = 4800
amodDelay = 5.5

def info():
    global mooring, cable, motorRunState
    "globals which may be externally set"
    print "(go:%s)   motor('%s')   cable=%.2f   mooring=%d   (depth():%.2f)" % \
        (go.isSet(), motorRunState, cable, mooring, depth())

def init():
    "set global vars to defaults"
    global ser,cable, mooring, motorRunState, go, motorOn
    cable = 0
    mooring = 20
    # motorRunState off, down, up
    motorRunState = 'off' 
    go = Event()
    motorOn = Event()
    ser = None
    ser = Serial(port=port,baudrate=baudrate,name=name,eol=eol)

def start():
    "start serial and reader thread"
    global go, buffOut
    buffOut = ''
    # threads run while go is set
    go.set()
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
        print "IOError on serial, calling buoy.stop() ..."
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
    buffOut += s
    

def motor(state):
    "set motorRunState to off, down, up; motorOn event"
    global ser, motorRunState, motorOn
    if state not in ( 'off', 'down', 'up'):
        return ser.log( "motor(up|off|down), not '%s'" % state )
    #
    motorRunState = state
    ser.log( "motor %s with cable at %.2f depth %.2f" % (state,cable,depth()) )
    if motorRunState=='off': motorOn.clear()
    else: motorOn.set()

def motorThread():
    "when motor is on: update cable, check dock and slack"
    global ser, go, cable, motorRunState, motorOn
    # motor could be on when emulation starts
    while go.isSet(): 
        motorOn.wait()
        motorLastTime = time()
        sleep(.1)
        # up
        if motorRunState=='up':
            # surfaced?
            if slack():
                motor('off')
                # no amod delay here, sleep(amodDelay) is in serThread
                ser.log( "buoy surfaced" )
                amodPut("#S,%s,00%s" % (buoyID, ser.eol))
            # simple linear
            cable += (time() - motorLastTime) * .331
            if slack(): # surfaced?
                motor('off')
                ser.log( "line slack, buoy surfaced" )
                # no amod delay here, sleep(amodDelay) is in serThread
                amodPut("#S,%s,00%s" % (buoyID, ser.eol))
        # down
        if motorRunState=='down':
            if docked():
                motor('off')
                # no amod delay here, sleep(amodDelay) is in serThread
                ser.log( "buoy docked" )
                amodPut("#S,%s,00%s" % (buoyID, ser.eol))
            # simple linear
            cable -= (time() - motorLastTime) * .2
            if docked():
                motor('off')
                cable=0
                ser.log( "buoy docked" )
                # no amod delay here, sleep(amodDelay) is in serThread
                amodPut("#S,%s,00%s" % (buoyID, ser.eol))

def slack():
    "determine if the cable is slack"
    return depth()<.1

def docked():
    "are we docked?"
    global cable
    return cable<.1

def depth():
    "mooring - cable, mod by current"
    global mooring, cable
    # TBD
    d = mooring-cable
    if d<0: return 0
    else: return d

#def modGlobals(**kwargs):
#    "change defaults from command line"
#    # change any of module globals, most likely mooring or cable
#    # ?? globals need defaults to reset on run()
#    if kwargs: 
#        # update module globals
#        glob = globals()
#        logmsg = "params: "
#        for (i, j) in kwargs.iteritems(): 
#            glob[i] = j
#            logmsg += "%s=%s " % (i, j)

init()
