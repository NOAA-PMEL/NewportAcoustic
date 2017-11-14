# emulate antenna module v4
import time
from laraSer import Serial
from serial.tools.list_ports import comports
from threading import Thread, Event, Timer
from design import *
import winch

# globals set in init(), start()
#  alarm    := triggered once per cycle, ignore if True
#  flag    := please watch, defaults from .ini
#  request  := input from buoy

name = 'antmod'
portSelect = 1      # select port 0-n of multiport serial
baudrate = 9600

CTD_DELAY = 0.53
CTD_WAKE = 0.78

sbe = {}
flag = {}
setting = {}
serThreadObj = None

def info():
    "globals which may be externally set"
    print "(go:%s)   syncMode=%s   syncModePending=%s   sleepMode=%s" % \
        (go.isSet(), syncMode, syncModePending, sleepMode)

def init():
    "set globals to defaults"
    global mooring__line, mooring, buoyLine, floatsLine, antLine
    mooring__line = mooring-(buoyLine+floatsLine+antLine)
    global go
    go = Event()
    flagInit()
    settingInit()
    sbe_init()

def parseIni(x):
    "read antmod.ini for settings"

def flagInit():
    global flag
    flag = { 
        'ice': False, 
        'surface': False, 
        'target': False, 
        'velocity': False,
        }
    # flags may be set by antmod.ini
    parseIni('flag')

def settingInit():
    global setting
    # defaults before .ini
    setting = {
        'ice': -1.3,
        'surface': 1.5,
        }
    parseIni('setting')

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
            if sbe['event'].isSet(): 
                sbe['event'].clear()
                sbe_process()
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
    global ser, sbe, setting, flag
    sbe['pending'] = False  # set True in sbe_req()
    sbe['depth'] = d = depth()
    sbe['temp'] = t = temp()
    #
    if flag['log']:
        ser.log("d %f.2, t %f.2") # log to file in antmod.c
        sbe_req()
    if flag['velocity']:
        velocity(d)         # calls sbe_req() if not done
    #
    if flag['depth']:
        flag['depth'] = False
        ser.putline("depth %.2f" % d)
    if flag['temp']:
        flag['temp'] = False
        ser.putline("temp %.2f" % d)
    #
    if phase!=2:            # other flags only valid in phase 2
        return  
    # monitoring sbe during ascent
    if flag['target']:
        if ( (phase==2 and d<request['target'])
            or (phase==4 and d>request['target'])):
            flag['target'] = False
            ser.putline("target %.2f" % d)
        else: sbe_req()
    if flag['ice']:
        if t<setting['ice']:
            flag['ice'] = False
            ser.putline("ice %.2f" % d)
            # will be reset before next ascent
        else: sbe_req()
    if flag['surface']:
        if d<setting['surface']:
            flag['surface'] = False
            phase(3)
            # will be reset before next ascent
        else: sbe_req()
#end def sbe_process():

def buoyProcess(l):
    "process serial input (from buoy)"
    # 'chars\r'
    global flag, setting, phase
    ls = l.split()
    if len(ls)==0: return
    cmd = ls[0]
    if 'phase'==cmd:
        phase = int(ls[1])
        phaseSetup(phase)
    elif 'file'==cmd:
        loadFile(name=ls[1], size=ls[2])
    elif 'target'==cmd:
        setting[cmd] = float(ls[1])
        flag[cmd] = True
    elif 'velocity'==cmd:
        setting[cmd] = int(ls[1])
        flag[cmd] = True
    elif 'depth'==cmd:
        flag[cmd] = True
        sbe_req()
    elif 'temp'==cmd:    
        flag[cmd] = True
        sbe_req()
#end def buoyProcess(l):

# pseudo static var
velTime=0
velDepth=0.0
def velocity(d):
    "process sbe reading for velocity"
    global velTime, velDepth # static vars in C
    global flag, setting
    if not velTime:
        # first reading, set start time depth
        velTime = time.time()
        velDepth = d
    else:
        if setting['velocity']==0:
            # down is positive velocity, increasing depth
            v = (d - velDepth) / (time.time() - velTime)
            ser.putline("velocity %.2f" % v)
            velTime = 0
            velDepth = 0.0
            flag['velocity'] = False
        else:
            # more data
            request['velocity'] -= 1
            sbe_req()
#end def velocity(d):

def phase(p):
    "phase transition"
    global phase, flag, setting, ser
    phase = p
    ser.putline("phase %d" % phase)
    ser.log("phase %d" % phase)
    if flag['velocity']:        # terminate pending velo
        setting['velocity'] = 0
        velocity(sbe['depth'])  # last depth
    flagInit()
    #
    if p==1:        # docked
        ser.log("docked. sleeping.")
    elif p==2:        # ascend
        k = setting.keys()
        for i in ('ice', 'surface', 'log'):
            if i in k: flag[i] = True
        if flag['log']: sbe_req()
    elif p==3:        # surface
        ser.putline("surfaced %.2f" % sbe.depth)
        gpsIrid()
        ser.putline("descend")
    elif p==4:        # descend
        k = setting.keys()
        for i in ('log'):
            if i in k: flag[i] = True
        if flag['log']: sbe_req()
#end def phase(p):

def sbe_init():
    "init"
    global sbe, CTD_DELAY
    sbe['ctdDelay'] = CTD_DELAY
    sbe['pending'] = False
    sbe['depth'] = 0.0
    sbe['temp'] = 0.0
    sbe['event'] = Event()

def sbe_req():
    "poke sbe to return data after a little time"
    global sbe, flag
    if sbe['pending']: return
    sbe['pending'] = True
    Timer(sbe['ctdDelay'], sbe['event'].set)

def gpsIrid():
    "pretend to send files"
    ser.putline("gps %.4f %.4f" % (1.2, 3.4))
    ser.putline("signal 5")
    ser.putline("connected")
    ser.putline("sent %d/%d %d" % (4,6,12345))
    ser.putline("descend")

def depth():
    "mooring-(cable+buoyL+floatsL+antL). always below surface, max antCTDpos"
    global mooring__line, antSBEpos
    dep = mooring__line-winch.cable()
    if dep<antSBEpos: return antSBEpos
    else: return dep

def temp():
    "return 20.1 unless we emulate ice at a certain depth"
    return 20.1

init()

# pseudo OO: obj_func()==obj.func() obj['var']==obj.var
