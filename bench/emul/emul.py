import sys, time
import laraSer, winch, buoy
from shared import logSafe


# program parameters
winchArgs = buoyArgs = radioArgs = {}
syncmode = 0


def init():
    "set initial values, open serials"
    global syncmode, phase, depth
    global ctdSer, iridSer, amodSer
    global winchArgs, buoyArgs, radioArgs

    # arguments
    for a in sys.argv[1:]:
        f = a.find('=')+1
        arg = a[f:]
        if '-?' in a:
            print "usage: -syncmode=y -cable=# -mooring=#" + \
                " -amodRate=# -notUsed=[ctd|irid|amod] -?"
        if '-s' in a: 
            syncmode=1
            print "syncmode on"
        elif '-c' in a:
            winchArgs['cableLen']=float(arg)
            print "cable %s" % arg
        elif '-m' in a:
            winchArgs['mooring']=int(arg)
            print "mooring %s" % arg
        elif '-a' in a:
            winchArgs['amodRate'] = float(arg)
            print "amodem rate %s" % arg
        elif '-n' in a:
            if 'ctd' in arg: 
                print "no ctd"
                buoyArgs['no'] = 1
            if 'irid' in arg: 
                print "no irid"
                radioArgs['no'] = 1
            if 'amod' in arg: 
                print "no amod"
                winchArgs['no'] = 1

    # initialize objects
    ctdSer = laraSer.Serial(port='/dev/ttyS7',baudrate=9600)
    ctdSer.name = 'ctd'
    iridSer = laraSer.Serial(port='/dev/ttyS8',baudrate=19200)
    iridSer.name = 'irid'
    if 'no' in buoyArgs.keys():
        ctdSer.close()
    if 'no' in radioArgs.keys():
        iridSer.close()
    winch.turnOn(**winchArgs)


def shut():
    "close down"
    winch.turnOff()

# phase=
# 1. Recording, locked to winch. <- depth = depthMax
# 2. Up. <- amod up command
# 3. iridium. <- depth = 0 or amod stop command
# 4. Down. <- amod down command


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
                    logSafe( "ctd: break ignored" )
                    # syncmode=0
                    # log "syncmode=n"
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



def ctdOut():
    "instrument sample"
    # "# 20.6538,  0.01145,    0.217,   0.0622, 01 Aug 2016 12:16:50"
    # "\r\n# t.t,  c.c,  d.d,  s.s,  dd Mmm yyyy hh:mm:ss\r\n"

    # ctd delay to process, nominal 3.5 sec. Add variance?
    time.sleep(3.5)
    # adjust depth
    depth = winch.mooring - winch.cable()
    ###
    d="01 Aug 2016"
    t="12:16:50"
    # note: modify temp for ice
    ctdSer.putline("\r\n# %f, %f, %f, %f, %s %s\r\n" %
        (20.1, 0.01, depth, 0.06, d, t))



if __name__=='__main__': init(); main(); shut()


# Notes:
# add exception handling for errs
