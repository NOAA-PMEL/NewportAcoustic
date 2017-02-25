import laraSer
import time

# serial ports
ctdSer = laraSer.Serial(port='/dev/ttyS7',baudrate=9600,timeout=0)
iridSer = laraSer.Serial(port='/dev/ttyS8',baudrate=19200,timeout=0)
amodSer = laraSer.Serial(port='/dev/ttyS9',baudrate=4800,timeout=0)

# program parameters
depth = depthMax = 30
phase=1

# phase=
# 1. Recording WISPR, locked to winch
# 2. Up. Watch for currents. Beware ice at surface. Keep tension.
# 3. iridium. Call into Satellite
# 4. Down. 

# init
syncmode=0
programStarted = phaseStarted = time.time()

def main():
    "main loop, check all ports and respond"
    global phase, syncmode, phaseStarted

    while true:
        # CTD  
        if ctdSer.in_waiting:
            # note: syncmode is special, a trigger not a command
            if syncmode: 
                c = ctdSer.read()
                if '\x00' in c: 
                    # break
                    syncmode=0
                else: 
                    ctdOut()
                    # flush
                    ctdSer.read(999)
            else:
                # upper case is standard for commands, but optional
                l = ctdSer.getline().upper()
                if 'TS' in l: ctdOut()
                elif 'SYNCMODE=Y' in l: syncmode=1

        # iridium radio
        if iridSer.in_waiting:
            l = iridSer.getline()

        # acoustic modem
        if amodSer.in_waiting:
            l = amodSer.getline()
            # up command
            if '#R,01,03' in l:
                phase=2
                phaseStarted=time.time()
            # down command
            elif '#F,01,00' in l:
                phase=4
                phaseStarted=time.time()
        


def ctdOut():
    "instrument sample"
    # "# 20.6538,  0.01145,    0.217,   0.0622, 01 Aug 2016 12:16:50"
    # "\r\n# t.t,  c.c,  d.d,  s.s,  dd Mmm yyyy hh:mm:ss\r\n"

    # ctd delay to process, nominal 3.5 sec. Add variance?
    time.sleep(3.5)
    # adjust depth
    winch()
    ###
    date="01 Aug 2016"
    time="12:16:50"
    # note: modify temp for ice
    ctdSer.putline("\r\n# %f, %f, %f, %f, %s %s\r\n" %
        20.1, 0.01, depth, 0.06, date, time)

def winch():
    "update (global) depth due to winch activity"
    global phaseStarted, phase, depth
    # 2do:  add complexity to match observed data
    #       add option for current
    #       add winch autostop for surface
    if phase=2:
        d = time.time() - phaseStarted) * 


# Notes:
# add exception handling for errs
