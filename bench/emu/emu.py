import sys
import time
import laraSer

# serial ports

# program parameters
depth = mooring = 30
syncmode=0
phase=1

# phase=
# 1. Recording, locked to winch. <- depth = depthMax
# 2. Up. <- amod up command
# 3. iridium. <- depth = 0 or amod stop command
# 4. Down. <- amod down command

def init():
    "set initial values, open serials"
    global syncmode, phase, depth, mooring
    global winchUpdate, programStarted
    global ctdSer, iridSer, amodSer
    ctdSer = laraSer.Serial(port='/dev/ttyS7',baudrate=9600)
    ctdSer.name = 'ctd'
    iridSer = laraSer.Serial(port='/dev/ttyS8',baudrate=19200)
    iridSer.name = 'irid'
    amodSer = laraSer.Serial(port='/dev/ttyS9',baudrate=4800)
    amodSer.name = 'amod'
    programStarted = winchUpdate = time.time()
    # arguments
    arg = sys.argv[1:]
    while arg:
        a=arg[0]
        if '-?' in a:
            print "usage: -syncmode -phase # -depth # -mooring #" + \
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
            mooring=float(arg[1])
            print "mooring %s" % depth
            arg = arg[2:]
        elif '-n' in a:
            if 'ctd' in arg[1]: 
                print "no ctd"
                ctdSer.close()
            if 'irid' in arg[1]: 
                print "no irid"
                iridSer.close()
            if 'amod' in arg[1]: 
                print "no amod"
                amodSer.close()
        else: arg = arg[1:]


def main():
    "main loop, check all ports and respond"
    global phase, winchUpdate, depth, depthMax, syncmode
    global ctdSer, iridSer, amodSer

    while 1:
        # CTD. syncmode, sample, settings
        if ctdSer.in_waiting:
            # syncmode is special, a trigger not a command, eol not required
            if syncmode: 
                c = ctdSer.get()
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
                ctdSer.put('S>')

        # iridium radio. TBD
        if iridSer.in_waiting:
            l = iridSer.getline(eol='\r')
            if '+PD' in l: 
                iridSer.put( 
                    ( '\r\n' 
                    + 'UTC Date=%02d-%02d-%04d'
                    + ' Satellites Used=%2d' 
                    + '\r\n' 
                    ) % (1, 1, 2017, 11) 
                )
            elif '+PT' in l: 
                iridSer.put( 
                    ( '\r\n' 
                    + 'UTC Time=%02d:%02d:%02d.%03d'
                    + ' Satellites Used=%2d' 
                    + '\r\n' 
                    ) % (1, 1, 1, 1, 11) 
                )
            elif '+PL' in l: 
                iridSer.put( 
                    ( '\r\n' 
                    + 'Latitude=%02d:%02d.%04d%s'
                    + ' Longitude=%03d:%02d.%04d%s'
                    + ' Altitude=%.1f meters'
                    + ' Satellites Used=%2d' 
                    + '\r\n' 
                    ) % (1, 1, 1, 'N', 1, 1, 1, 'E', -0.1, 11) 
                )
            elif '+CPAS' in l: 
                iridSer.put(
                    ( '\r\n' 
                    + '+CPAS:%03d'
                    + ' Satellites Used=%2d' 
                    + '\r\n' ) 
                    % (1, 11)
                )
            elif l: pass
                    

        # acoustic modem. up, stop, down.
        if amodSer.in_waiting:
            l = amodSer.getline()
            # up command
            if '#R,01,03' in l:
                phase=2
                print "phase=2, up"
                winchUpdate=time.time()
            # stop command
            elif '#S,01,00' in l:
                phase=3
                print "phase=3, iridium"
                winchUpdate=time.time()
            # down command
            elif '#F,01,00' in l:
                phase=4
                print "phase=4, down"
                winchUpdate=time.time()
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
    global winchUpdate, phase, depth, mooring
    # 2do:  add complexity to match observed data
    #       add option for current
    #       phase 1=docked, 2=up, 3=stopped, 4=down
    t = time.time()
    if phase==2:
        # up, simple linear
        depth -= .331 * (t - winchUpdate) 
        if depth<0: 
            # at surface
            depth=0
            phase=3
    if phase==4:
        # down, simple linear
        depth += 0.2 * (t - winchUpdate) 
        if depth>mooring: 
            # docked in winch
            depth=mooring
            phase=1
    winchUpdate = t


if __name__=='__main__': init(); main()


# Notes:
# add exception handling for errs
