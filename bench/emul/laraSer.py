import serial
from threading import Lock
from string import split, join
import time

class Serial(serial.Serial):
    "extra methods to handle our serial ports"

    def __init__(self, eol='\r', eol_out='', name=None, *args, **kwargs):
        # buff is for input not consumed by getline
        super(Serial, self).__init__(*args, **kwargs)
        # getline() buffers partial line input
        self.buff=''
        # putQ() buffers output, to emulate slow comm
        self.logLevel=2
        self.eol=eol
        self.eol_out=eol_out
        self.timeout = ( 1 / self.baudrate ) * 3
        if name: self.name = name

    # serial may not be thread safe, use lock in class var
    ioLock = Lock()

    def logSafe(self,s):
        "Log to stdout, thread safe"
        Serial.ioLock.acquire()
        print s
        Serial.ioLock.release()

    def logIn(self,s):
        "Log to stdout, thread safe"
        if self.logLevel>0:
            self.logSafe( "%s<< %r" % (self.name, s) )

    def logOut(self,s):
        "Log to stdout, thread safe"
        if self.logLevel>0:
            self.logSafe( "%s>> %r" % (self.name, s) )

    def log(self,s):
        "Log to stdout, thread safe"
        if self.logLevel>1:
            # time seconds sss.sss
            stamp = time.time()
            self.logSafe( "%s: %s %s" % 
                (self.name, "%.3f" % (stamp%1000), s) )

    def get(self):
        "Get all available chars"
        if self.in_waiting:
            c = self.read(self.in_waiting)
        b = self.buff + c
        self.buff = ''
        self.logIn(b)
        return b

    def getline(self, echo=0):
        "Get full lines from serial, keep eol; partial to self.buff"
        eol = self.eol
        eol_out = self.eol_out
        # read chars
        if self.in_waiting:
            c = self.read(self.in_waiting)
            if echo: 
                # translate if echo & self.eol_out
                # note - only works if eol is one char (i.e. \r->\r\n)
                if eol_out and (eol in c): 
                    c = join(split(c, eol), eol_out)
                self.write(c)
            b = self.buff + c
            if eol in b:
                i = b.find(eol) + len(eol)
                r = b[:i]
                self.buff = b[i:]
                self.logIn(r)
                return r
            else: 
                # partial
                self.buff = b
                return ''

    def putline(self, s):
        "put to serial"
        eol = self.eol_out
        self.write("%s%s" % (s, eol))
        self.logOut(s)

    def put(self,s):
        "put to serial"
        self.write(s)
        self.logOut(s)
