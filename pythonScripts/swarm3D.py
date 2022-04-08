#!/usr/bin/env python
# -*- coding: utf-8 -*-a

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import json,ast
import os
import coordinatesSweep as cs 

from mpl_toolkits.mplot3d import Axes3D



fig = plt.figure()
ax1 = p3.Axes3D(fig) #creo il grafico

ax1.set_xlim3d([0.0,5000.0])
ax1.set_xlabel("X")

ax1.set_ylim3d([0.0,5000.0])
ax1.set_ylabel("Y")

ax1.set_zlim3d([0.0,5000.0])
ax1.set_zlabel("Z")

#ax2 = p3.Axes3D(fig) #subgraf per punti di arrivo

cs.sweep() #converto il file mat in json

with open("drones.json", "r") as f: #Apro il file json e estraggo il dizionario contenente le posizioni dei droni
	data = json.load(f)
	data = ast.literal_eval(json.dumps(data))
	f.close()
with open("destinationPoints.json", "r") as f: #Apro il file json e estraggo il dizionario contenente i punti di arrivo
	destination = json.load(f)
	destination = ast.literal_eval(json.dumps(destination))
	f.close()
with open("intruders.json", "r") as f: #Apro il file json e estraggo il dizionario contenente le posizioni degli intrusi
	intruders = json.load(f)
	intruders = ast.literal_eval(json.dumps(intruders))
	f.close()
with open("intrDestination.json", "r") as f: #Apro il file json e estraggo il dizionario le destinazioni degli intrusi
	intrDest = json.load(f)
	intrDest = ast.literal_eval(json.dumps(intrDest))
	f.close()
				

numDataPoints = len(data["drone1"]["x"])

nDestPoints = len(destination["point1"]["x"])

nIntruders = len(intruders["intruder1"]["x"])

#per ogni drone disegno la sua traiettoria
dronePoint = [ax1.plot(data[k]["x"],data[k]["y"],data[k]["z"],marker = "h", markeredgecolor="black")[0] for k in data]
#intrPoint = [ax1.plot(intruders[k]["x"],intruders[k]["y"],intruders[k]["z"],marker = "h", markeredgecolor="black")[0] for k in intruders]


def droneAnimation(num, dataSet, lines):
	for k,line in zip(dataSet,lines):
		for i in range(num):
			x = dataSet[k]["x"]
			y = dataSet[k]["y"]
			z = dataSet[k]["z"]
			line.set_data(x[num],y[num])
			line.set_3d_properties(z[num])
			line.set_color("blue");
	return lines

def intrAnimation(num, dataSet, lines):
	for k,line in zip(dataSet,lines):
		for i in range(num):
			x = dataSet[k]["x"]
			y = dataSet[k]["y"]
			z = dataSet[k]["z"]
			line.set_data(x[num],y[num])
			line.set_3d_properties(z[num])
			line.set_color("red");
	return lines




drone_anim = animation.FuncAnimation(fig, droneAnimation, frames = numDataPoints, fargs = (data, dronePoint), interval = 5, repeat = False) 
#intr_anim = animation.FuncAnimation(fig, intrAnimation, frames = nIntruders, fargs = (intruders, intrPoint), interval = 5, repeat = False) 
plt.show()


