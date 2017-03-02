import sys
import time
import laraSer

# serial ports

# program parameters
depth = depthMax = -30
syncmode=0
phase=1

# phase=
# 1. Recording, locked to winch. <- depth = depthMax
# 2. Up. <- amod up command
# 3. iridium. <- depth = 0 or amod stop command
# 4. Down. <- amod down command

def init():
    "set initial values, open serials"
    global phase, phaseStarted, programStarted
    global ctdSer, iridSer, amodSer
    ctdSer = laraSer.Serial(port='/dev/ttyS7',baudrate=9600,timeout=0.3)
    ctdSer.name = 'ctd'
    iridSer = laraSer.Serial(port='/dev/ttyS8',baudrate=19200,timeout=0.3)
    iridSer.name = 'irid'
    amodSer = laraSer.Serial(port='/dev/ttyS9',baudrate=4800,timeout=0.6)
    amodSer.name = 'amod'
    programStarted = phaseStarted = time.time()
    # arguments
    arg = sys.argv[1:]
    while arg:
        a=arg[0]
        if '-?' in a:
            print "usage: -syncmode -phase # -depth # -max #" + \
                " -no [ctd|irid|amod] -?"
        if '-s' in a: 
            syncmode=1
            print "syncmode on"
            arg = arg[1:]
        elif '-p' in a:
            phase=int(arg[1])
            print "phase %s" % phase
            arg = arg[2:]
        elif '-d' in a:
            depth=float(arg[1])
            print "depth %s" % depth
            arg = arg[2:]
        elif '-m' in a:
            maxDepth=float(arg[1])
            print "maxDepth %s" % depth
            arg = arg[2:]
        elif '-n' in a:
            if 'ctd' in arg[1]: ctdSer.close()
            if 'irid' in arg[1]: iridSer.close()
            if 'amod' in arg[1]: amodSer.close()
        else: arg = arg[1:]


def main():
    "main loop, check all ports and respond"
    global phase, phaseStarted, depth, depthMax, syncmode
    global ctdSer, iridSer, amodSer

    init()

    while 1:
        # CTD. syncmode, sample, settings
        if ctdSer.in_waiting:
            # syncmode is special, a trigger not a command, eol not required
            if syncmode: 
                time.sleep(.1)
                c = ctdSer.read(999)
                print "ctd> %r" % c
                if '\x00' in c: 
                    # break
                    print "ctd: break ignored"
                    # syncmode=0
                    # print "syncmode=n"
                if '\x00' != c:
                    ctdOut()
                    # flush
                    ctdSer.reset_input_buffer
            # command line. note: we don't do timeout
            else:
                # upper case is standard for commands, but optional
                l = ctdSer.getline().upper()
                if 'TS' in l: ctdOut()
                elif 'SYNCMODE=Y' in l: 
                    syncmode=1
                else: pass
                ctdSer.write('S>')

        # iridium radio. TBD
        if iridSer.in_waiting:
            l = iridSer.getline()

        # acoustic modem. up, stop, down.
        if amodSer.in_waiting:
            l = amodSer.getline()
            # up command
            if '#R,01,03' in l:
                phase=2
                print "phase=2, up"
                phaseStarted=time.time()
            # stop command
            elif '#S,01,00' in l:
                phase=3
                print "phase=3, iridium"
                phaseStarted=time.time()
            # down command
            elif '#F,01,00' in l:
                phase=4
                print "phase=4, down"
                phaseStarted=time.time()
            # something strange
            elif l:
                errOut("amod: unexpected %r" % l)


def errOut(s="unexpected err"):
    print "Err: %s" % s
        


def ctdOut():
    "instrument sample"
    # "# 20.6538,  0.01145,    0.217,   0.0622, 01 Aug 2016 12:16:50"
    # "\r\n# t.t,  c.c,  d.d,  s.s,  dd Mmm yyyy hh:mm:ss\r\n"

    # ctd delay to process, nominal 3.5 sec. Add variance?
    time.sleep(3.5)
    # adjust depth
    winch()
    ###
    d="01 Aug 2016"
    t="12:16:50"
    # note: modify temp for ice
    ctdSer.putline("\r\n# %f, %f, %f, %f, %s %s\r\n" %
        (20.1, 0.01, depth, 0.06, d, t))


def winch():
    "update (global) depth due to winch activity"
    global phaseStarted, phase, depth
    # 2do:  add complexity to match observed data
    #       add option for current
    #       phase 1=docked, 2=up, 3=stopped, 4=down
    if phase==2:
        # up, simple linear
        d = .331 * (time.time() - phaseStarted) 
        depth += d
        if depth>0: 
            # at surface
            depth=0
            phase=3
            phaseStarted=time.time()
    if phase==4:
        # down, simple linear
        d = -0.2 * (time.time() - phaseStarted) 
        depth += d
        if depth<depthMax: 
            # docked in winch
            depth=depthMax
            phase=1
            phaseStarted=time.time()

class Winch(mooring=-30, cable=0, phase=1, motor=0):
    "Winch object"
    startTime=updateTime=0.0

    def update():
        "update cable length"
        # rise, simple linear
        if phase==2:
            t = time.time()
            cable += .331 * (t - updateTime)
            updateTime = t
            if slack():
                # note: should adjust partial 
                motor = 0
                phase = 3
        # fall, simple linear
        if phase==4:
            t = time.time()
            depth -= .200 * (t - updateTime)
            updateTime = t
            if depth<mooring:
                depth = mooring
                motor = 0
                phase = 1

    def rise():
        "buoy rises"
        startTime=updateTime=time.time()
        print "winch: rise %s" % time.ctime()
        motor = 1
        phase = 2

    def fall():
        "buoy falls"
        startTime=updateTime=time.time()
        print "winch: fall %s" % time.ctime()
        motor = 1
        phase = 2

main()


# Notes:
# add exception handling for errs
