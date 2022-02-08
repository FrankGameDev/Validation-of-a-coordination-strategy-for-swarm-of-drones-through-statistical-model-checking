#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""
Genero un file json contenente tutti i movimenti effettuati dai droni dello sciame
"""

import DyMat as dm
import json

#n_drones = numero droni
def sweep():
	#Importa il file .mat e estrapola i dati richiesti
	d = dm.DyMatFile('System_res.mat')
	
	# da implementare: n_drones = d.data("K.N").item();
	n = 0
	with open("constants.mo","r") as f:
		line = f.readlines()
		for x in line:
			i = x.find("constant Integer N")
			if (i > -1):
				n = int(x.split(";")[0][-1])
				print(n)
			

	drones=dict()


	for i in range(n):
		drones["drone"+str(i+1)] = {}

	for i in range(n):
		
		x = d.data("drone.x["+str(i+1)+"]") #Estraggo le coordinate del drone i-esimo
		y = d.data("drone.y["+str(i+1)+"]")
		z = d.data("drone.z["+str(i+1)+"]")

		#Converti le coordinate in float e li inserisce in una lista 
		xS = [value.item() for value in x.astype(float)]
		yS = [value.item() for value in y.astype(float)]
		zS = [value.item() for value in z.astype(float)]

		drones["drone"+str(i+1)].update({"x":xS, "y":yS, "z":zS}) #Creo e estendo il dizionario che conterrà tutte le coordinate di tutti i droni

	with open("simulation.json", "w") as f:
		json.dump(drones, f)
		f.close()






