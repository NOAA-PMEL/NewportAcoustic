import sys, time
import laraSer, winch, buoy
from shared import *


# program parameters
mooring = 30
syncmode=0
phase=1

def init():
    "set initial values, open serials"
    global iridSer

    winchArgs = {}
    buoyArgs = {}
    radioArgs = {}
    # arguments
    for a in sys.argv[1:]:
        f = a.find('=')+1
        arg = a[f:]
        if '-?' in a:
            print "usage: -syncmode=y -cable=# -mooring=#" + \
                " -amodRate=# -notUsed=[ctd|irid|amod] -?"
        if '-s' in a: 
            buoyArgs['syncmode'] = 1
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
    iridSer = laraSer.Serial(port='/dev/ttyS8',baudrate=19200)
    iridSer.name = 'irid'
    if not 'no' in buoyArgs.keys():
        buoy.turnOn(**buoyArgs)
    if 'no' in radioArgs.keys():
        iridSer.close()
    if not 'no' in winchArgs.keys():
        winch.turnOn(**winchArgs)


def shut():
    "close down"
    winch.turnOff()
    buoy.turnOff()

# phase=
# 1. Recording, locked to winch. <- depth = depthMax
# 2. Up. <- amod up command
# 3. iridium. <- depth = 0 or amod stop command
# 4. Down. <- amod down command


def main():
    "main loop, check all ports and respond"
    global iridSer

    while 1:
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



if __name__=='__main__': init(); main(); shut()


# Notes:
# add exception handling for errs
