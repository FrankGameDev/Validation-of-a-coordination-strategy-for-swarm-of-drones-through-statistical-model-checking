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

#animazione dei dorni
def droneAnimation(num, dataSet, lines):
	for k,line in zip(dataSet,lines):
		for i in range(num):
			x = dataSet[k]["x"]
			y = dataSet[k]["y"]
			z = dataSet[k]["z"]
			line.set_data(x[num],y[num])
			line.set_3d_properties(z[num])
	return lines	

#animazione dei punti di arrivo
def destinationAnim(num, datas, points):
	for k,point in zip(datas,points):
		for i in range(num):
			x = datas[k]["x"]
			y = datas[k]["y"]
			z = datas[k]["z"]
			point.set_data(x,y)
			point.set_3d_properties(z)
	return points

fig = plt.figure()
ax1 = p3.Axes3D(fig) #creo il grafico

ax1.set_xlim3d([0.0,50.0])
ax1.set_xlabel("X")

ax1.set_ylim3d([0.0,50.0])
ax1.set_ylabel("Y")

ax1.set_zlim3d([0.0,50.0])
ax1.set_zlabel("Z")

#ax2 = p3.Axes3D(fig) #subgraf per punti di arrivo

cs.sweep() #converto il file mat in json

with open("simulation.json", "r") as f: #Apro il file json e estraggo il dizionario
	data = json.load(f)
	data = ast.literal_eval(json.dumps(data))
	f.close()
with open("destinationPoints.json", "r") as f: #Apro il file json e estraggo il dizionario
	destination = json.load(f)
	destination = ast.literal_eval(json.dumps(destination))
	f.close()
				

numDataPoints = len(data["drone1"]["x"])

nDestPoints = len(destination["point1"]["x"])

#per ogni drone disegno la sua traiettoria
lines = [ax1.plot(data[k]["x"],data[k]["y"],data[k]["z"],marker = "o" )[0] for k in data]

#per ogni punto di arrivo, marco il punto sul plot
#destpoint = [ax2.plot(destination[k]["x"],destination[k]["y"],destination[k]["z"], linestyle = "", marker = "x")[0] for k in destination]



for k in destination:
	ax1.scatter(destination[k]["x"],destination[k]["y"],destination[k]["z"],marker = "x" )

drone_anim = animation.FuncAnimation(fig, droneAnimation, frames = numDataPoints, fargs = (data,lines), interval=50, blit=True, repeat = False)
#dest_anim = animation.FuncAnimation(fig, destinationAnim, frames = nDestPoints, fargs=(destination,destpoint), interval = 50, blit=True, repeat= False)
plt.show()


