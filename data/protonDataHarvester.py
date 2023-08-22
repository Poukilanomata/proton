#coding: utf8

import os
import time
from datetime import datetime
import serial
from signal import signal
from signal import SIGTERM
import sys
import matplotlib.pyplot as plt
import numpy as np

arduino = serial.Serial(port='/dev/cu.ESP32', baudrate=115200)

if arduino:
   print("Vodka Putin. Proton 3 connecté.")

# Création des fichiers, directions et méthode d'écriture
try:
   os.mkdir(os.getcwd()+'/proton3_flydata')
except FileExistsError:
   pass
os.chdir(os.getcwd()+'/proton3_flydata')
now = datetime.now()
nowText = now.strftime("/%d_%m_%Y_%H-%M-%S")
fileNameCI = './'+nowText+"_CI"+".txt"
fileNameSG = './'+nowText+"_SG"+".txt"
fileNameSD = './'+nowText+"_SD"+".txt"
fileNameCA =  './'+nowText+"_CA"+".txt"

with open(fileNameCI, "w") as myFile:
       pass
with open(fileNameSG, "w") as myFile:
       pass
with open(fileNameSD, "w") as myFile:
       pass
with open(fileNameCA, "w") as myFile:
       pass

def addToFile(fileName, text):
   with open(fileName, "a") as myFile:
       myFile.write(text)

def readFile(fileName):
   with open(fileName, "r") as myFile:
       return myFile.read()

# Gestion de l'interruption
def handler(sig, frame):
   global notStop
   notStop = False

signal(SIGTERM, handler)


# Data processing

def processData(rawData):
    l = rawData.split(";")
    if len(l) == 0:
        return None, None, None
    if l[1] == "I":
        fileName = fileNameCI
    elif l[1] == "G":
        fileName = fileNameSG
    elif l[1] == "D":
        fileName = fileNameSD
    elif l[1] == "A":
        fileName = fileNameCA
    else:
        return None, None, None
    time = l[0]
    data = l[2:]
    return fileName, data, time

# Main loop
notStop = True
try:
    while notStop:
        # Acquisition et traitement de la donnée arduino
        rawData = arduino.readline().decode("utf-8")
        rawData = rawData[:-1]
        fileName, data, time = processData(rawData)
        if data == None:
            continue
            print("Error data")
        print("Receiving...")
        text = ';'.join(data)
        try:
            addToFile(fileName, time+";"+text+"\n")
        except TypeError:
            pass
except KeyboardInterrupt:
    print()

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

    curveNameCI = fileNameCI
    curveNameSG = fileNameSG
    curveNameSD = fileNameSD
    curveNameCA = fileNameCA

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
            c+=1
    convert(dataSD)
    processServo(dataSD)

    curvesCA = readFile(curveNameCA) 
    lines = curvesCA.split('\n')
    lines = lines[:-2]
    dataCA = [[] for i in range(len(lines[0].split(';')))]
    for line in lines:
        elements = line.split(';')
        c = 0
        for element in elements:
            dataCA[c].append(element)
            c+=1
    convert(dataCA)

    fig, axs = plt.subplots(4)
    fig.suptitle('Proton 3 data')
    axs[0].plot(dataCI[0], dataCI[1], '+', label='yaw')
    axs[0].plot(dataCI[0], dataCI[2], '+', label='pitch')
    axs[0].plot(dataCI[0], dataCI[3], '+', label='roll')
    axs[1].plot(dataSG[0], dataSG[1], '+')
    axs[2].plot(dataSD[0], dataSD[1], '+')
    axs[3].plot(dataCA[0], dataCA[1], '+', label='x')
    axs[3].plot(dataCA[0], dataCA[2], '+', label='y')
    axs[3].plot(dataCA[0], dataCA[3], '+', label='z')

    axs[0].legend()
    axs[3].legend()

    plt.show()

    sys.exit(0)