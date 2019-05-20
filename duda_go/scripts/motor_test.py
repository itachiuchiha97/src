import serial
import struct
ard = serial.Serial('/dev/ttyUSB1',9600)

while True:
    ans = int(input("Enter Speed-"))
    # ans = chr(ans)
    # my_ans = ans.encode("latin")
    my_ans = struct.pack("B",ans)
    print(my_ans)
    ard.write(my_ans)

ard.close()
