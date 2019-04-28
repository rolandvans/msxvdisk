import serial.tools.list_ports
import serial
import time
import sys

#ports = list(serial.tools.list_ports.comports())
port=[
    p.device
    for p in serial.tools.list_ports.comports()
    if 'CH340' or 'Arduino' in p.description
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
	#arduino = serial.Serial(port[0], 1000000, timeout=30)
	arduino = serial.Serial(port[0], 1000000, timeout=None)
except:
	print ("Failed to connect on Arduino.")
	sys.exit(1)

# open 2 disk images
try:
    #disk=[open('driveb.dsk','rb+'),open('ease_or.dsk','rb+')]
    disk=[open('ease_or.dsk','rb+'),open('driveb.dsk','rb+')]
except:
    print ("Failed to open disk image files.")
    sys.exit(1)

#main loop, wait for  commands from Arduino
while (1):
    print("Ok")
    cmd=arduino.read(4)
    t0=time.clock()
    # write 'W'
    if (cmd[0]==87):
        disknr=cmd[1]
        sector=cmd[2]+256*cmd[3]
        if (sector>=0 and sector<1440):
            print("Disk:",disknr,"Write sector:",sector)
            data=arduino.read(512)
            disk[disknr].seek(512*sector)
            disk[disknr].write(data)
        else:
            print("\nInvalid sector write:",sector)

    # read 'R'
    if (cmd[0]==82):
        disknr=cmd[1]
        sector=cmd[2]+cmd[3]*256
        if (sector>=0 and sector<1440):
            disk[disknr].seek(512*sector)
            data=disk[disknr].read(512)
            t1=time.clock()
            arduino.write(data)
            t2=time.clock()
            print("Disk:",disknr,"Read sector:",sector)
            print("dt read:{0}ms. dt write:{1}ms".format((t1-t0)*1000,(t2-t1)*1000))
        else:
            print("Invalid sector read:",sector)

    # comment 'C'
    if (cmd[0]==67):
        comment=arduino.readline()
        print(comment.decode(encoding='UTF-8'))
#close handles and exit
disk[0].close()
disk[1].close()
arduino.close()
sys.exit()
