import sys, time
import laraSer, winch, buoy, radio
from shared import *


# program parameters
buoyArgs = {}
winchArgs = {}
radioArgs = {}

def init():
    "set initial values"
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
    if buoyArgs: buoy.modGlobals(**buoyArgs)
    if winchArgs: winch.modGlobals(**winchArgs)
    if radioArgs: radio.modGlobals(**winchArgs)

def run():
    "start up"
    buoy.run()
    radio.run()
    winch.run()

def stop():
    "close down"
    buoy.stop()
    radio.stop()
    winch.stop()



if __name__=='__main__': init(); run()


# Notes:
# add exception handling for errs
