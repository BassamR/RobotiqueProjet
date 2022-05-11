from cgi import test
import serial
import numpy as np

ser = serial.Serial("COM3", 115200)
ser.timeout = None

angleArray = []
numOfSamples = 20

while(1):
    r = ser.readline()
    #r = r[0:len(r)-1] #to get rid of \n
    #print(r)
    angleArray.append(int(r))
    if(len(angleArray) > numOfSamples+1):
        print("finished")
        break

sum = 0
for i in range(len(angleArray)):
    sum += angleArray[i]

average = sum/len(angleArray)
print("This is average", average)
