import laraSer
import time

# serial ports
ctdSer = laraSer.Serial(port='/dev/ttyS7',baudrate=9600,timeout=0)
iridSer = laraSer.Serial(port='/dev/ttyS8',baudrate=19200,timeout=0)
amodSer = laraSer.Serial(port='/dev/ttyS9',baudrate=4800,timeout=0)

# program parameters
depth = depthMax = 30
phase=1

# phase=
# 1. Recording, locked to winch. <- depth = depthMax
# 2. Up. <- amod up command
# 3. iridium. <- depth = 0 or amod stop command
# 4. Down. <- amod down command

# init
syncmode=0
programStarted = phaseStarted = time.time()

def main():
    "main loop, check all ports and respond"
    global phase, syncmode, phaseStarted

    while true:
        # CTD. syncmode, sample, settings
        if ctdSer.in_waiting:
            # note: syncmode is special, a trigger not a command
            if syncmode: 
                c = ctdSer.read(999)
                if '\x00' in c: 
                    # break
                    print "ctd> break"
                    syncmode=0
                else: 
                    print "ctd> %s" % c
                    ctdOut()
                    # flush
                    ctdSer.read(999)
            else:
                # upper case is standard for commands, but optional
                l = ctdSer.getline().upper()
                if 'TS' in l: ctdOut()
                elif 'SYNCMODE=Y' in l: syncmode=1
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
                phaseStarted=time.time()
            # stop command
            elif '#S,01,00' in l:
                phase=3
                phaseStarted=time.time()
            # down command
            elif '#F,01,00' in l:
                phase=4
                phaseStarted=time.time()
            else:
                errOut("amod: unexpected '%s'" % l)


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
    date="01 Aug 2016"
    time="12:16:50"
    # note: modify temp for ice
    ctdSer.putline("\r\n# %f, %f, %f, %f, %s %s\r\n" %
        20.1, 0.01, depth, 0.06, date, time)


def winch():
    "update (global) depth due to winch activity"
    global phaseStarted, phase, depth
    # 2do:  add complexity to match observed data
    #       add option for current
    if phase=2:
        # up, simple linear
        d = .331 * (time.time() - phaseStarted) 
        depth += d
        if depth>0: 
            # at surface
            depth=0
            phase=3
            phaseStarted=time.time()
    if phase=4:
        # down, simple linear
        d = -0.2 * (time.time() - phaseStarted) 
        depth += d
        if depth>depthMax: 
            # docked in winch
            depth=depthMax
            phase=1
            phaseStarted=time.time()


# Notes:
# add exception handling for errs
