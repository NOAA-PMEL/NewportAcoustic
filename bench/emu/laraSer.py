import serial
import sys

class Serial(serial.Serial):
    def __init__(self, *args, **kwargs):
        # buff is for input not consumed by getline
        self.buff=''
        self.log=1
        super(Serial, self).__init__(*args, **kwargs)

    def getline(self, eol='\n'):
        "Get full lines from serial, record. \
            Empty q -> None, no full line -> '', eol -> line"
        if self.in_waiting:
            # read chars
            b = self.buff + self.read(self.in_waiting)
            if eol in b:
                i = b.find(eol) + len(eol)
                l = b[:i]
                self.buff = b[i:]
                # record
                if self.log:
                    # \r in case of partial (below)
                    print "\r%s> %r" % (self.name, l)
                return l
            else: 
                # partial
                self.buff = b
                # record
                if self.log:
                    # if not a full line, \r to column 0
                    sys.stdout.write( "\r%s..%s" % (self.name, b))
                return ''

    def putline(self, s):
        "put to serial, record"
        self.write("%s\r\n" % s)
        # record
        if self.log:
            print "%s< %r" % (self.name, s)
