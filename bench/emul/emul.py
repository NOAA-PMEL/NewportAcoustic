# emulator. v3
import winch, sbe16, sbe39, floats
from design import mooring

def init():
    "init all"
    sbe16.init()
    winch.init()
    sbe39.init()

def start():
    "start all"
    sbe16.start()
    winch.start()
    sbe39.start()

def stop():
    "stop all"
    sbe16.stop()
    winch.stop()
    sbe39.stop()

def info():
    "info all"
    sbe16.info()
    winch.info()
    sbe39.info()
    
def restart():
    "stop init start"
    stop()
    init()
    start()

#import sys
#def my_except_hook(exctype, value, traceback):
#    if exctype == KeyboardInterrupt:
#        print "stopping..."
#        stop()
#    else:
#        sys.__excepthook__(exctype, value, traceback)
#sys.excepthook = my_except_hook

import atexit
atexit.register(stop)

if __name__=='__main__': 
    start()
else: 
    print "start() stop() restart() sbe16.info() mooring=30 winch.cable(0)"
    info()
