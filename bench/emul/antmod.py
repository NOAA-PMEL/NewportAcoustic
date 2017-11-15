# emulate antenna module v4
import time
from laraSer import Serial
from serial.tools.list_ports import comports
from threading import Thread, Event, Timer
from design import *
import winch

# globals set in init(), start()

name = 'antmod'
portSelect = 1      # select port 0-n of multiport serial
baudrate = 9600

CTD_DELAY = 0.53
CTD_WAKE = 0.78

sbe = {}
flags = {}           # watch sbe depth/temp
value = {}          # depth/temp
setting = {}        # algorithm settings
serThreadObj = None

flagsSet = [
    (), # p0
    (), # p1
    ('log', 'surface'), # p2
    (), # p3
    (), # p4
    ]

sbeInit = {
    'depth': 0,
    }

flagsInit = { 
    'depth': False,
    'ice': False, 
    'log': False,
    'surface': False, 
    'target': False, 
    'temp': False,
    'velocity': False,
    }

valueInit ={
    'ice': -1.3,
    'surface': 1.5,
    }

sbeInit={
    'ctdDelay': CTD_DELAY,
    'pending': False,
    'depth': 0.0,
    'temp': 0.0,
    'timer': None,
    }

def info():
    "globals which may be externally set"
    print "(go:%s)   syncMode=%s   syncModePending=%s   sleepMode=%s" % \
        (go.isSet(), syncMode, syncModePending, sleepMode)

def init():
    "set globals to defaults"
    global mooring__line, mooring, buoyLine, floatsLine, antLine
    mooring__line = mooring-(buoyLine+floatsLine+antLine)
    global go, phase, flags, value, sbe
    go = Event()
    phase = 2
    flags = flagsInit.copy()
    value = valueInit.copy()
    sbe = sbeInit.copy()

def start(portSel=portSelect):
    "start I/O thread"
    global go, serThreadObj, name, ser
    # threads run while go is set
    go.set()
    try:
        # select port 0-n of multiport serial
        port = comports()[portSel].device
        ser = Serial(port=port,baudrate=baudrate,name=name)
    except: 
        print "no serial for %s" % name
        ser = None
    serThreadObj = Thread(target=serThread)
    serThreadObj.daemon = True
    serThreadObj.name = name
    serThreadObj.start()

def stop():
    global go, serThreadObj
    "stop threads"
    if not serThreadObj: return
    go.clear()
    # wait until thread ends, allows daemon to close clean
    serThreadObj.join(3.0)
    if serThreadObj.is_alive(): 
        print "stop(): fail on %s" % serThreadObj.name

def serThread():
    "thread: loop looks for serial input; to stop set sergo=0"
    global go, sbe, ser
    if not ser.is_open: ser.open()
    ser.buff = ''
    #try:
    if True:
        while go.isSet():
            #if sbe['event'].isSet(): 
            #    sbe['event'].clear()
            #    sbe_process()
            l = ser.getline()
            # getline returns None, '', or 'chars\r'
            if l: buoyProcess(l)
    #except:
    #    print "Error, stop()"
    #    stop()
    if ser.is_open: ser.close()
#end def serThread():

def sbe_process():
    "called when sbe['event'].isSet(), consume pseudo-data from sbe"
    global ser, sbe, value, flags, phase
    sbe['pending'] = False  # set True in sbe_req()
    sbe['depth'] = d = depth()
    sbe['temp'] = t = temp()
    #
    if flags['log']:
        ser.log("d %f.2, t %f.2") # log to file in antmod.c
    if flags['velocity']:
        velocity(0)         # calls sbe_req() if not done
    #
    if flags['depth']:
        flags['depth'] = False
        ser.putline("depth %.2f" % d)
    if flags['temp']:
        flags['temp'] = False
        ser.putline("temp %.2f" % d)
    # 
    if flags['target']:
        if d<value['target']:
            flags['target'] = False
            ser.putline("target %.2f" % d)
    if flags['ice']:
        if t<value['ice']:
            flags['ice'] = False
            ser.putline("ice %.2f" % d)
    if flags['surface']:
        if d<=value['surface']:
            flags['surface'] = False
            ser.putline("surfaced %.2f" % sbe['depth'])
            phaseChange(3)
    if any( flags.values() ):
        sbe_req()
#end def sbe_process():

def buoyProcess(l):
    "process serial input (from buoy)"
    # 'chars\r'
    global flags, value, phase
    ls = l.split()
    if len(ls)==0: return
    cmd = ls[0]
    if 'phase'==cmd:
        phase = int(ls[1])
        phaseChange(phase) # reset flags, values
    elif 'file'==cmd:
        loadFile(name=ls[1], size=ls[2])
    elif 'depth'==cmd:
        flags[cmd] = True
        sbe_req()
    elif 'temp'==cmd:    
        flags[cmd] = True
        sbe_req()
    elif 'target'==cmd:
        value[cmd] = float(ls[1])
        flags[cmd] = True
        sbe_req()
    elif 'velocity'==cmd:
        velocity( int(ls[1]) )
        flags[cmd] = True
        sbe_req()
#end def buoyProcess(l):

def velocity(c=0):
    "c>0 start average, c<0 end average, else continue average"
    global flags, sbe
    d = sbe['depth']
    if c>0:
        # start average
        velocity.time = time.time()
        velocity.depth = d
        velocity.count = c-1
        sbe_req()
    elif c<0 or velocity.count==0:
        # done. down is positive velocity, increasing depth
        v = (d - velocity.depth) / (time.time() - velocity.time)
        ser.putline("velocity %.2f" % v)
        flags['velocity'] = False
    else:
        # more data
        velocity.count -= 1
        sbe_req()
# static var
velocity.time=0
velocity.depth=0
velocity.count=0
#end def velocity(d):

def phaseChange(p):
    "phase transition, reset flags and values"
    global phase, flags, flagsInit, flagsSet, value, valueInit, ser
    phase = p
    ser.putline("phase %d" % phase)
    ser.log("phase %d" % phase)
    if flags['velocity']:        # terminate pending velo
        velocity(-1)
    flags = flagsInit.copy()
    value = valueInit.copy()
    ser.log( flagsSet[p] )
    for k in flagsSet[p]:        # flags for this phase
        flags[k] = True
    #
    if p==1:        # docked
        None
    elif p==2:        # ascend
        None
    elif p==3:        # surface
        gpsIrid()
    elif p==4:        # descend
        None
    if any( flags.values() ):
        sbe_req()
#end def phaseChange(p):

def sbe_req():
    "poke sbe to return data after a little time"
    global sbe, flags
    if sbe['pending']: return
    sbe['pending'] = True
    sbe['timer'] = Timer(sbe['ctdDelay'], sbe_process)
    sbe['timer'].start()

from datetime import datetime
def gpsIrid():
    "pretend to send files"
    # UTC Time=20:25:44.000
    t=datetime.now()
    s=t.strftime("gps %H:%M:%S.%f")
    ser.putline(s[:-3])
    ser.putline("signal 5")
    ser.putline("connected")
    ser.putline("sent %d/%d %d" % (4,6,12345))
    ser.putline("done")

def depth():
    "mooring-(cable+buoyL+floatsL+antL). always below surface, max antSBEpos"
    global mooring__line, antSBEpos
    dep = mooring__line-winch.cable()
    if dep<antSBEpos: return antSBEpos
    else: return dep

def temp():
    "return 20.1 unless we emulate ice at a certain depth"
    return 20.1

init()

# pseudo OO: obj_func()==obj.func() obj['var']==obj.var
