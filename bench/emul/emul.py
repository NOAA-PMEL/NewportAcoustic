import sys, time
import laraSer, winch, buoy, ant
from shared import *

def init():
    "init all"
    buoy.init()
    winch.init()
    ant.init()

def start():
    "start all"
    buoy.start()
    winch.start()
    ant.start()

def stop():
    "stop all"
    buoy.stop()
    winch.stop()
    ant.stop()

def info():
    "info all"
    buoy.info()
    winch.info()
    ant.info()
    
def restart():
    "stop init start"
    stop()
    init()
    start()

if __name__=='__main__': 
    init()
    start()
else: 
    print "init() start() stop() buoy.info() winch.mooring=14 winch.cable=15"


# Notes:
# add exception ^C stop() before exit
