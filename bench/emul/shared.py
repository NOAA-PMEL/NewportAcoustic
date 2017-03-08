from threading import Lock

# global ID
buoyID = '00'
winchID = '01'

# laraSer uses threads, include lock for output
ioLock=Lock()

def logSafe(s):
    "Log to stdout, thread safe"
    ioLock.acquire()
    print s
    ioLock.release()

