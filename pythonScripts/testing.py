import os
import time
import os.path
from EBStop import EBStop 
import numpy as np
from OMPython import OMCSessionZMQ
import math
import json

os.system("rm -f ./System")      # .... to be on the safe side


omc = OMCSessionZMQ()
omc.sendExpression("getVersion()")
omc.sendExpression("cd()")

omc.sendExpression("loadModel(Modelica)")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"connectors.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"constants.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"randgen.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"extFunction.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"drone/drone.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"Monitors/MonitorSuccess.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"Monitors/MonitorCollision.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"algorithms/flockingModule.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"algorithms/psoController.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"algorithms/collisionAvoidance.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"drone/controller.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"drone/setPoint.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"faultSystem.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"intruders/intruders.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"intruders/intrudersController.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"intruders/intrudersPoint.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"intruders/rockets.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"intruders/rocketController.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"intruders/rocketPoint.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"intruders/staticObs.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"system.mo\")")
omc.sendExpression("getErrorString()")

startTime = time.time()
noFault = "{{1, 0, 0, 0},{1, 0, 0, 0},{1, 0, 0, 0},{1, 0, 0, 0}}"

#Scrive il file contenente le informazioni sotto forma di json
def writeData(name,collDD, collDO, arrived, dTime):
	diz = {"collDD": collDD, "collDO": collDO, "arrived": arrived, "arrivalTime":dTime}
	with open("Simulation_Data/SimulationData_" + str(name) + ".json", "wt") as f:
		json.dump(diz, f)
		f.flush()
		f.close()

#Modifica i parametri della simulazione e compila 
def parameterSweep(n, intr, miss, statObs, fault, flyZone):
	os.system("rm LogOverride.txt")

	omc.sendExpression("setParameterValue(K, N," + str(n) +")")
	omc.sendExpression("setParameterValue(K, nIntr," + str(intr) +")")
	omc.sendExpression("setParameterValue(K, nRocket," + str(miss) +")")
	omc.sendExpression("setParameterValue(K, nStatObs," + str(statObs) +")")
	omc.sendExpression("setParameterValue(K, flyZone,fill(" + str(flyZone) +",3))")
	omc.sendExpression("setParameterValue(faultSys, transMatrix," + str(fault) +")")

	omc.sendExpression("buildModel(System, stopTime=180, simflags=\"-overrideFile=newValues.txt\")")
	omc.sendExpression("getErrorString()")

#Simula il sistema restituendo i KPI
def startSimulation(flyZone):

	randX, randY, randZ = np.random.random_sample(), np.random.random_sample(),np.random.random_sample()
	randX = math.fabs(flyZone * randX - flyZone)
	randY = math.fabs(flyZone * randY - flyZone)
	randZ = math.fabs(flyZone * randZ - flyZone)
	with open("newValues.txt", 'wt') as f:
		f.write("p.rand[1]="+str(randX)+"\n")
		f.write("p.rand[2]="+str(randY)+"\n")
		f.write("p.rand[3]="+str(randZ)+"\n")
		f.flush()
		os.fsync(f)

	os.system("./System -overrideFile=newValues.txt >> LogOverride.txt")
	#extract the stop time of the simulation
	vars = omc.sendExpression("readSimulationResult(\"System_res.mat\",{time})")
	omc.sendExpression("getErrorString()")
	stopTime = int(vars[-1][-1]) 

	ndrone = int(omc.sendExpression("val(const.N," + str(stopTime) + ", \"System_res.mat\")"))
	#Dizionario contenente tempo di arrivo e stato finale di ogni drone
	droneInfo = dict.fromkeys([x for x in range(1,ndrone+1)], ()) 

	""" 	print("("+str(omc.sendExpression("val(p.setx[1]," + str(stopTime) + ", \"System_res.mat\")")) + "," +
			str(omc.sendExpression("val(p.sety[1]," + str(stopTime) + ", \"System_res.mat\")")) + "," +
			str(omc.sendExpression("val(p.setz[1]," + str(stopTime) + ", \"System_res.mat\")")) + ")\n" ) """

	#extract collision occurance
	tDD = omc.sendExpression("val(colMan.tDD," + str(stopTime) + ", \"System_res.mat\")") #Collisioni droni
	tDC = omc.sendExpression("val(colMan.tDC," + str(stopTime) + ", \"System_res.mat\")") #Collisioni droni-intrusi
	tDR = omc.sendExpression("val(colMan.tDR," + str(stopTime) + ", \"System_res.mat\")") #Collisioni droni-missili
	tDSC = omc.sendExpression("val(colMan.tDSC," + str(stopTime) + ", \"System_res.mat\")") #Collisioni droni-ostacoli statici
	for j in range(1,ndrone+1):
		arrivalTime = omc.sendExpression("val(sucMo.arrivalTime[" + str(j) + "]," + str(stopTime) + ", \"System_res.mat\")")
		droneArrived = omc.sendExpression("val(sucMo.arrived[" + str(j) + "]," + str(stopTime) + ", \"System_res.mat\")")
		droneInfo[j] = (droneArrived, arrivalTime)

	os.system("rm -f System_res.mat")      # .... to be on the safe side
	os.system("rm -f newValues.txt")      # .... to be on the safe side
	return (tDD, tDC, tDR, tDSC, droneInfo) 

#Esegue simulazioni del sistema con ogni tipo di scenario
def getSimulationData():
	with open("Simulation_Data/LogEBS.txt", "wt") as f:
		f.write("Tempo computazionale per ogni scenario attraverso EBS\n")
		f.flush()
		os.fsync(f)

	os.system("mkdir -p Simulation_Data")

	drones = [4,10,20]
	intruders = 10
	missile = 10
	staticObs = 8
	flyZone = [100,150,200]
	noFault = "{{1, 0, 0, 0},{1, 0, 0, 0},{1, 0, 0, 0},{1, 0, 0, 0}}"
	faultMatrix = "{{0.7, 0.1, 0.1, 0.1},{0.6, 0.4, 0, 0},{0.5, 0, 0.5, 0},{0.7, 0, 0, 0.3}}"

	
	nf = 0
	nomeF = "no"
	for fault in ([noFault, faultMatrix]):
		print(fault)
		if(nf>0): nomeF = "si"
		for d in drones:
			for zone in flyZone:
		
				startTime = time.time()

				print("simulazione con: (" + str(d) +" droni, area di volo: "+ str(zone) +")\n")
				#lista collisioni per ogni iterazione 
				collDD, collDC, collDR, collDSC = [], [], [], []

				#Lista totale collisioni ostacoli per ogni iterazione
				totalCollisionObs = list()

				ebsDD, ebsDO, ebsArrived, ebsTime = EBStop(), EBStop(), EBStop(), EBStop()

				arrivedDrone, arrivalTime = list(), list()
				tmptime = [[] for _ in range(d)]


				#Otterranno l'output dell'algoritmo di stopping, così da determinare l'interruzione della simulazione
				stoppingDD, stoppingDO, stoppingDArrived, stoppingTime = False, False, False, False
				index = 0
				parameterSweep(d,intruders,missile,staticObs,fault,zone)
				while((not(stoppingDD and stoppingDO and stoppingDArrived and stoppingTime))):
					(tmpDD, tmpDC, tmpDR, tmpDSC, droneInf) = startSimulation(zone)
					collDD.append(tmpDD)
					collDC.append(tmpDC)
					collDR.append(tmpDR)
					collDSC.append(tmpDSC)
					totalCollisionObs.append(collDC[-1] + collDR[-1] + collDSC[-1])
					cont = 0
					tmpList = list()
					droneIndex = 1
					#Salvo numero di droni arrivati
					for x in droneInf.keys():
						if(droneInf[x][0] > 0.0): #se il drone è arrivato
							cont += 1
							tmpList.append(droneInf[x][1])
							tmptime[droneIndex-1].append(droneInf[x][1])
						droneIndex += 1

					arrivedDrone.append(cont)
					#Memorizzo tempo di arrivo medio
					arrivalTime.append(np.sum(tmpList)/len(tmpList))
					# print("Tempo di arrivo medio: "+ str(arrivalTime[-1])+ "\n")
					# print("droni arrivati all'iterazione " + str(index) + " = "  + str(arrivedDrone[-1])+"\n")
					# print("collisioni dd: " ,collDD, "\nobs total: " , totalCollisionObs , "\n")
					if(index > 3):
						stoppingDD = True if(stoppingDD) else ebsDD.find_stop_value(collDD, max(collDD) - min(collDD))
						stoppingDO = True if(stoppingDO) else ebsDO.find_stop_value(totalCollisionObs, max(totalCollisionObs) - min(totalCollisionObs))
						stoppingDArrived = True if(stoppingDArrived) else ebsArrived.find_stop_value(arrivedDrone, max(arrivedDrone) - min(arrivedDrone))
						stoppingTime = True if(stoppingTime) else ebsTime.find_stop_value(arrivalTime, max(arrivalTime) - min(arrivalTime))
					# print("informazioni iterazione num " + str(index) + ": Collisioni tra droni --> " + str(tmpDD) + "\t"+
					# 	" Collisioni DtoO --> " + str(totalCollisionObs[-1]) +"\n stoppingDD: " + str(stoppingDD) + " stoppingDO: " + str(stoppingDO) + 
					# 	" stoppingArrived: " + str(stoppingDArrived) + " stoppingTime: "+ str(stoppingTime) +"\n")
					index+=1
					print(index) 
				print("Tempo di esecuzione = ", (time.time() - startTime))
				droneArrivalTime = dict.fromkeys([i for i in range(1,d+1)],[])
				for i in range(len(droneArrivalTime.keys())):
					droneArrivalTime[i+1] = tmptime[i]
				print(droneArrivalTime)
				#Scrivo i risultati all'interno di un file json
				writeData(str(d) + "_" + str(intruders) + "_" +  str(missile) + "_" + str(staticObs) + "_" + str(nomeF) +
						"_" + str(zone), collDD, totalCollisionObs, arrivedDrone, droneArrivalTime)
				
				#Salvo in un file il tempo di simulazione di ogni ebs
				with open("Simulation_Data/LogEBS.txt", "a") as f:
					f.write("Simulazione " +str(d) + "_" + str(intruders) + "_" +  str(missile) + "_" + str(staticObs) + "_" + str(nomeF) +
						"_" + str(zone)+ ". Tempo di esecuzione: " + str(time.time()-startTime) + "; Iterazioni EBS: " + str(index) + ";\n")
					f.flush()
					os.fsync(f)
		nf+=1

getSimulationData()
with open("Simulation_Data/LogEBS.txt", "a") as f:
	f.write("Tempo di esecuzione totale = " + str(time.time()-startTime)+";\n")
	f.flush()
	os.fsync(f)
print("Execution time = ", (time.time()-startTime))
