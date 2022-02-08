#!/usr/bin/env python
# -*- coding: utf-8 -*-a

import matplotlib.pyplot as plt
import numpy as np
import scipy.io as spy
import pandas as pd
import json,ast
import os
import coordinatesSweep as cs 

from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d') #creo il grafico

cs.sweep() #converto il file mat in json

with open("simulation.json", "r") as f: #Apro il file json e estraggo il dizionario
	data = json.load(f)
	data = ast.literal_eval(json.dumps(data))
	f.close()


#Per ogni drone disegno la sua traiettoria
for k in data:	
	for i in range(0,len(data[k]["x"]),5):
		ax.scatter(data[k]["x"][i],data[k]["y"][i],data[k]["z"][i])


plt.show()


