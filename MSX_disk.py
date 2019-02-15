import serial.tools.list_ports
import serial
import time
import sys

#ports = list(serial.tools.list_ports.comports())
port=[
    p.device
    for p in serial.tools.list_ports.comports()
    if 'Arduino' in p.description
]
# we need to find exactly one port
nr=len(port)
if (nr<1):
    print ("No Arduino found.")
    sys.exit(1)
if (nr>1):
    print ("Multiple Arduino's found.")
    sys.exit(1)

# open the port 
try:
	arduino = serial.Serial(port[0], 1000000)
except:
	print ("Failed to connect on Arduino.")
	sys.exit(1)

# open a file
try:
   	f=open('DRIVEA.dsk','rb+')
except:
    print ("Failed to open disk image file.")
    sys.exit(1)

while (1):
    print("waiting for command")
    cmd=arduino.read(4)
    # write 'W'
    if (cmd[0]==87):
        disk=cmd[1]
        sector=cmd[2]+256*cmd[3]
        if (sector>=0 and sector<1440):
            print("Disk:",disk,"Write sector:",sector)
            data=arduino.read(512)
            # disable writing for now...
            f.seek(512*sector)
            f.write(data)
        else:
            print("Invalid sector write:",sector)

    # read 'R'
    if (cmd[0]==82):
        disk=cmd[1]
        sector=cmd[2]+cmd[3]*256
        if (sector>=0 and sector<1440):
            f.seek(512*sector)
            data=f.read(512)
            arduino.write(data)
            print("Disk:",disk,"Read sector:",sector)
        else:
            print("Invalid sector read:",sector)

    if (cmd[0]==67 and cmd[1]==0):
        code=cmd[2]+cmd[3]*512
        print(cmd[2],cmd[3])

f.close()
arduino.close()
sys.exit()
