import sys, time
import laraSer, winch, buoy
from shared import *


# program parameters
buoyArgs = {}
winchArgs = {}
radioArgs = {}
iridSer = None

def init():
    "set initial values"
    global iridSer
    global winchArgs, buoyArgs, radioArgs
    # arguments
    for a in sys.argv[1:]:
        f = a.find('=')+1
        arg = a[f:]
        if '-?' in a:
            print "usage: -syncmode=y -cable=# -mooring=#" + \
                " -amodDelay=# -?"
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
            winchArgs['amodDelay'] = float(arg)
            print "amodem delay %s" % arg

    # initialize objects
    iridSer = laraSer.Serial(port='/dev/ttyS8',baudrate=19200)
    iridSer.name = 'irid'
    if buoyArgs: buoy.modGlobals(**buoyArgs)
    if winchArgs: winch.modGlobals(**winchArgs)

def open():
    "start up"
    buoy.open()
    winch.open()

def shut():
    "close down"
    winch.shut()
    buoy.shut()

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



if __name__=='__main__': init(); open(); main(); shut()


# Notes:
# add exception handling for errs
