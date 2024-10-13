from math import sin
import matplotlib.pyplot as plt

ts = 0.002

startFreq = 0
endFreq = 250

freqSweepRate = 0.01

freq = startFreq

freqVals = []

currTime = 0

while(freq < endFreq):
    setpoint = sin(freq * currTime) * 25000
    freq += freqSweepRate
    currTime += ts
    freqVals.append(setpoint)

print(currTime)
    
plt.plot(freqVals)
plt.show()
