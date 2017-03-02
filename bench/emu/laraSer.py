import serial
import sys

class Serial(serial.Serial):
    def __init__(self, *args, **kwargs):
        # buff is for input not consumed by getline
        super(Serial, self).__init__(*args, **kwargs)
        self.buff=''
        self.log=1
        self.timeout = ( 1 / self.baudrate ) * 3

    def get(self):
        "Get some chars"
        b = self.buff
        self.buff = ''
        while 1:
            c = self.read()
            if not c: break
            b += c
        # record
        if self.log:
            # \r in case of partial 
            print "\r%s> %r" % (self.name, b)
        return b

    def getline(self, eol='\n'):
        "Get full lines from serial, record. \
            Empty q -> None, no full line -> '', eol -> line"
        if self.in_waiting:
            # read chars
            b = self.buff + self.read(self.in_waiting)
            if eol in b:
                i = b.find(eol) + len(eol)
                r = b[:i]
                self.buff = b[i:]
            else: 
                # partial
                self.buff = b
                r = ''
            # record
            if self.log:
                print "%s> %r" % (self.name, r)
            return r

    def putline(self, s):
        "put to serial, record"
        self.write("%s\r\n" % s)
        # record
        if self.log:
            print "%s< '%s\\r\\n'" % (self.name, s)

    def put(self,s):
        "put to serial, record"
        self.write("%s" % s)
        # record
        if self.log:
            print "%s< '%s'" % (self.name, s)
