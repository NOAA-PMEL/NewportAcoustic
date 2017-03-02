import time
import serial

def syncm(s9,sl=1):
    s9.write("\r\n")
    time.sleep(4)
    s9.reset_input_buffer()
    s9.write("syncmode=y\r")
    t=time.time()
    time.sleep(sl)
    print "sleep  %.2f" % (time.time() - t)
    s9.write("qs\r")
    time.sleep(.1)
    print s9.read(99)
    query(s9)

def query(s9,sl=5):
    st=s9.timeout
    s9.timeout=sl
    t=time.time()
    s9.reset_input_buffer()
    s9.write("\r\n")
    print s9.readline()
    print s9.readline()
    print "elapse: %.2f" % (time.time() - t)
    s9.timeout=st

def elapse(f):
    t=time.time()
    apply(f,args)
    print "elapse %s" % (time.time()-t)

def brtime(s9,br=1):
    t=time.time()
    # s9.send_break(br)
    s9.break_condition=1
    time.sleep(br)
    s9.break_condition=0
    print "break %.2f" % (time.time() - t)
    time.sleep(1); t+=1
    s9.write('\n')
    while not s9.in_waiting: pass
    print "resp  %.2f" % (time.time() - t)
    print s9.readline()
    print s9.readline()
    print "reads %.2f" % (time.time() - t)
