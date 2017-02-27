import serial

class Serial(serial.Serial):
    def __init__(self, *args, **kwargs):
        # buff is for input not consumed by getline
        self.buff=''
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
                print "%s> %s" % (self.name, l)
                return l
            else: 
                self.buff = b
                return ''

    def putline(self, s):
        "put to serial, record"
        self.write("%s\r\n" % s)
        # record
        print "%s< %s" % (self.name, s)
