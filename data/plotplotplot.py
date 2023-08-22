import matplotlib.pyplot as plt
import sys
import numpy as np

# Full (gyro, servo, accelerometer, time)

def readFile(fileName):
    with open(fileName, "r") as myFile:
        return myFile.read()

def processServo(data):
    borneSup = 160
    borneInf = 20
    i = 0
    for element in data[1]:
        if int(element) <= borneInf:
            data[1][i] = borneInf
        if int(element) >= borneSup:
            data[1][i] = borneSup
        i+=1

def convert(data):
    for i in range(len(data)):
        for j in range(len(data[i])):
            try:
                data[i][j] = float(data[i][j])
            except ValueError:
                data[i][j] = 0

curveNameCI = "./proton3_flydata/{}_CI.txt".format(sys.argv[1])
curveNameSG = "./proton3_flydata/{}_SG.txt".format(sys.argv[1])
curveNameSD = "./proton3_flydata/{}_SD.txt".format(sys.argv[1])
curveNameCA = "./proton3_flydata/{}_CI.txt".format(sys.argv[1])

curvesCI = readFile(curveNameCI) 
lines = curvesCI.split('\n')
lines = lines[:-2]
dataCI = [[] for i in range(len(lines[0].split(';')))]
for line in lines:
    elements = line.split(';')
    c = 0
    for element in elements:
        dataCI[c].append(element)
        c+=1
convert(dataCI)

curveSG = readFile(curveNameSG) 
lines = curveSG.split('\n')
lines = lines[:-2]
dataSG = [[] for i in range(len(lines[0].split(';')))]
for line in lines:
    elements = line.split(';')
    c = 0
    for element in elements:
        dataSG[c].append(element)
        c+=1
convert(dataSG)
processServo(dataSG)

curveSD = readFile(curveNameSD) 
lines = curveSD.split('\n')
lines = lines[:-2]
dataSD = [[] for i in range(len(lines[0].split(';')))]
for line in lines:
    elements = line.split(';')
    c = 0
    for element in elements:
        dataSD[c].append(element)
        print(element)
        c+=1
convert(dataSD)
processServo(dataSD)

curvesCA = readFile(curveNameCA) 
lines = curvesCA.split('\n')
lines = lines[:-2]
dataCA = [[] for i in range(len(lines[0].split(';')))]
for line in lines:
    #print(line)
    elements = line.split(';')
    c = 0
    for element in elements:
        dataCA[c].append(element)
        c+=1
convert(dataCA)

fig, axs = plt.subplots(4)
fig.suptitle('Proton 3 data')
#axs[0].yaxis.set_visible(False)
axs[0].plot(dataCI[0], dataCI[1], '+', label='yaw')
axs[0].plot(dataCI[0], dataCI[2], '+', label='pitch')
axs[0].plot(dataCI[0], dataCI[3], '+', label='roll')
#axs[1].yaxis.set_visible(False)
print(dataSG[0])
print(dataSG[1])
axs[1].plot(dataSG[0], dataSG[1], '+')
#axs[2].yaxis.set_visible(False)
axs[2].plot(dataSD[0], dataSD[1], '+')
#axs[3].yaxis.set_visible(False)
axs[3].plot(dataCA[0], dataCA[1], '+', label='x')
axs[3].plot(dataCA[0], dataCA[2], '+', label='y')
axs[3].plot(dataCA[0], dataCA[3], '+', label='z')

axs[0].legend()
axs[3].legend()

plt.show()





