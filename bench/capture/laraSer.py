import serial
from threading import Lock

class Serial(serial.Serial):
    "extra methods to handle our serial ports"

    def __init__(self, eol='\n', name=None, *args, **kwargs):
        # buff is for input not consumed by getline
        super(Serial, self).__init__(*args, **kwargs)
        # getline() buffers partial line input
        self.buff=''
        # putQ() buffers output, to emulate slow comm
        self.logLevel=2
        self.eol=eol
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
            self.logSafe( "%s: %s" % (self.name, s) )

    def get(self):
        "Get some chars"
        b = self.buff
        self.buff = ''
        while 1:
            c = self.read()
            if not c: break
            b += c
        self.logIn(b)
        return b

    def getline(self, eol=''):
        "Get full lines from serial, strip eol; partial to self.buff"
        if eol == '': eol = self.eol
        if self.in_waiting:
            # read chars
            b = self.buff + self.read(self.in_waiting)
            if eol in b:
                i = b.find(eol)
                r = b[:i]
                i += len(eol)
                self.buff = b[i:]
                self.logIn(r)
            else: 
                # partial
                self.buff = b
                r = ''
            return r

    def putline(self, s, eol=''):
        "put to serial"
        if eol == '': eol = self.eol
        self.write("%s%s" % (s, eol))
        self.logOut(s)

    def put(self,s):
        "put to serial"
        self.write(s)
        self.logOut(s)
