import serial.tools.list_ports
for i in serial.tools.list_ports.comports(): print i.device
