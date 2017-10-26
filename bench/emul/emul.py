# emulator. v3
import winch, buoy, ant, floats
from design import mooring

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
    start()
else: 
    print "start() stop() restart() buoy.info() mooring=30 winch.cable(0)"
    info()


# Notes:
# add exception ^C stop() before exit
