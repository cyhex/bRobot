import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import defaultdict
from matplotlib.lines import Line2D

ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
sample_len = 10

fig = plt.figure()
ax0 = fig.add_subplot(4,1,1) # two rows, one column, first plot
ax0.set_ylabel(('Gyro - deg/sec'))
ax0.set_ylim(-20, 20)
lineGyro, = ax0.plot([0]*sample_len,color='red')
#ax0.add_line(lineGyro)


ax1 = fig.add_subplot(4,1,2)
ax1.set_ylabel('Accelerometer')
ax1.set_ylim(-20, 20)
lineAcc, = ax1.plot([0]*sample_len,color='red')

ax2 = fig.add_subplot(4,1,3)
ax2.set_ylabel('Angle')
ax2.set_ylim(-20, 20)
lineAgle, = ax2.plot([0]*sample_len,color='blue')

ax3 = fig.add_subplot(4,1,4)
ax3.set_ylabel('Torque')
ax3.set_ylim(-255, 255)
lineSpeed, = ax3.plot([0]*sample_len,color='blue')

def update(data):
    lineGyro.set_ydata(data['gyro'])
    lineAcc.set_ydata(data['acc'])
    lineAgle.set_ydata(data['angle'])
    lineSpeed.set_ydata(data['speed'])
    return lineGyro,lineAcc,lineAgle

def readLine(data):
    line = ser.readline()

    line  = line.strip()
    
    if not line.startswith('PID'):
        print line
    
    if not line.startswith('plot'):
        return data
    
    bits = line.split(":")
    if len(bits) == 5:
        data['gyro'].append(bits[1])
        data['acc'].append(bits[2])
        data['angle'].append(bits[3])
        data['speed'].append(bits[4])
        
    return data

def data_gen1():
    data = defaultdict(list)
    while 1:
        try:
            data = readLine(data)
        except Exception, e:
            continue
        
        if len(data['acc']) == sample_len + 1 :
            data['acc'].pop(0)
            data['gyro'].pop(0)
            data['angle'].pop(0)
            data['speed'].pop(0)
            yield data
        
   
ani = animation.FuncAnimation(fig, update, data_gen1, interval=100)
plt.show()

