import sys
import time
import laraSer

# serial ports

# program parameters
depth = depthMax = 30
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
    amodSer = laraSer.Serial(port='/dev/ttyS9',baudrate=4800,timeout=0.3)
    amodSer.name = 'amod'
    programStarted = phaseStarted = time.time()

def main():
    "main loop, check all ports and respond"
    global phase, phaseStarted, depth, depthMax, syncmode
    global ctdSer, iridSer, amodSer

    init()

    while 1:
        # CTD. syncmode, sample, settings
        if ctdSer.in_waiting:
            # note: syncmode is special, a trigger not a command
            if syncmode: 
                c = ctdSer.read(999)
                if '\x00' in c: 
                    # break
                    print "ctd> break"
                    syncmode=0
                    print "syncmode=n"
                else: 
                    print "ctd> %s" % c
                    ctdOut()
                    # flush
                    ctdSer.read(999)
            else:
                # upper case is standard for commands, but optional
                l = ctdSer.getline().upper()
                if 'TS' in l: ctdOut()
                elif 'SYNCMODE=Y' in l: 
                    syncmode=1
                    print "syncmode=y"
                else: pass

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
            else:
                errOut("amod: unexpected %r" % l)
                errOut("amod: buff %r" % amodSer.buff)


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
        if depth>depthMax: 
            # docked in winch
            depth=depthMax
            phase=1
            phaseStarted=time.time()


# start
arg = sys.argv[1:]
while arg:
    if '-s' in arg[0]: 
        syncmode=1
        print "syncmode on"
        arg = arg[1:]
    elif '-p' in arg[0]:
        phase=arg[1]
        print "phase %s" % phase
        arg = arg[2:]
    elif '-d' in arg[0]:
        depth=arg[1]
        print "depth %s" % depth
        arg = arg[2:]
    else: arg = arg[1:]

main()


# Notes:
# add exception handling for errs
