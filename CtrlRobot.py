import serial
import getch
serialport = serial.Serial ("/dev/ttyS0")
serialport.baudrate = 115200
while(True):
    x = getch.getch()
    
    if x == 'q' :
         break
    
    elif x == 'e' :
         str="+200+20015+00"
    
    elif x == 'd' :
         str="-200-20015+00"
         
    elif x == 'f' :
         str="+200-20015+00"
         
    elif x == 's' :
         str="-200+20015+00"
         
    else :
         str="+000+00015+00"
         
    
    serialport.write(str.encode())
