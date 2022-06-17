import os
import time
import os.path
from EBStop import EBStop 
import numpy as np
from OMPython import OMCSessionZMQ


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

omc.sendExpression("buildModel(System, stopTime=180)")
omc.sendExpression("getErrorString()")

startTime = time.time()
num_pass = 0
num_fail = 0

drones = [4,10,20]
intruders = [0,5]
missile = [0,5]
staticObs = [0,5]
flyZone = [100,150,200]

noFault = [[1, 0, 0, 0],[1, 0, 0, 0],[1, 0, 0, 0],[1, 0, 0, 0]]

faultMatrix = [[0.7, 0.1, 0.1, 0.1],[0.6, 0.4, 0, 0],[0.5, 0, 0.5, 0],[0.7, 0, 0, 0.3]]


#Simula il sistema con i parametri inseriti come argomenti della funzione, restituendo i KPI
def startSimulation(n, intr, miss, statObs, fault, flyZone):

	os.system("rm LogOverride.txt")

	#Overwrite parameters
	with open("newValues.txt", 'wt') as f:
		f.write("*.const.N="+str(n)+"\n")
		f.write("*.const.nIntr="+str(intr)+"\n")
		f.write("*.const.nRocket="+str(miss)+"\n")
		f.write("*.const.nStatObs="+str(statObs)+"\n")
		for i in range(len(fault)):
			for j in range(len(fault[i])):
				f.write("fault.transMatrix["+str(i+1)+","+str(j+1)+"]="+str(fault[i][j])+"\n")
		for i in range(3):
			f.write("const.flyZone["+str(i+1)+"]="+str(flyZone)+"\n")
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
		
	print(droneInfo, tDD) 
		
	print("Execution time = ", (time.time()-startTime))
	os.system("rm -f newValues.txt")      # .... to be on the safe side
	return (tDD, tDC, tDR, tDSC, droneInfo) 

#Usare un algoritmo EBSTOP per determinare tempo di stop
def manageSimulation():
	drones = [4,10,20]
	intruders = [0,5]
	missile = [0,5]
	staticObs = [0,5]
	flyZone = [100,150,200]
	noFault = [[1, 0, 0, 0],[1, 0, 0, 0],[1, 0, 0, 0],[1, 0, 0, 0]]
	faultMatrix = [[0.7, 0.1, 0.1, 0.1],[0.6, 0.4, 0, 0],[0.5, 0, 0.5, 0],[0.7, 0, 0, 0.3]]

	#lista collisioni per ogni iterazione
	collDD, collDC, collDR, collDSC = [], [], [], []

	#Lista totale collisioni ostacoli per ogni iterazione
	totalCollisionObs = list()	


	for d in drones:
		for intr in intruders:
			for miss in missile:
				for stat in staticObs:
					for zone in flyZone:
						print("simulazione con: (" + str(d) +","+ str(intr) +","+ str(miss) +","+ str(stat) +","+ str(zone) +")\n")
						#lista collisioni per ogni iterazione
						collDD, collDC, collDR, collDSC = [], [], [], []

						#Lista totale collisioni ostacoli per ogni iterazione
						totalCollisionObs = list()

						ebsDD, ebsDO, ebsArrived = EBStop(), EBStop(), EBStop()

						arrivedDrone = list()

						#Otterranno l'output dell'algoritmo di stopping, così da determinare l'interruzione della simulazione
						stoppingDD, stoppingDO, stoppingDArrived, stoppingTime = False, False, False, True
						index = 0
						while((not(stoppingDD and stoppingDO and stoppingDArrived and stoppingTime))):
							(tmpDD, tmpDC, tmpDR, tmpDSC, droneInf) = startSimulation(7,1,1,1,noFault,100)
							collDD.append(tmpDD)
							collDC.append(tmpDC)
							collDR.append(tmpDR)
							collDSC.append(tmpDSC)
							totalCollisionObs.append(collDC[-1] + collDR[-1] + collDSC[-1])
							cont = 0
							for x in droneInf.keys():
								if(droneInf[x][0] > 0.0): cont +=1
							arrivedDrone.append(cont)
							print("droni arrivati all'iterazione " + str(index) + " = "  + str(arrivedDrone[-1])+"\n")
							print("collisioni dd: " ,collDD, "\nobs total: " , totalCollisionObs , "\n")
							if(index > 0):
								stoppingDD = True if(stoppingDD) else ebsDD.find_stop_value(collDD, max(collDD) - min(collDD))
								stoppingDO = True if(stoppingDO) else ebsDO.find_stop_value(totalCollisionObs, max(totalCollisionObs) - min(totalCollisionObs))
								stoppingDArrived = True if(stoppingDArrived) else ebsArrived.find_stop_value(arrivedDrone, max(arrivedDrone) - min(arrivedDrone))
							print("informazioni iterazione num " + str(index) + ": Collisioni tra droni --> " + str(tmpDD) + "\t"+
								" Collisioni DtoO --> " + str(totalCollisionObs[-1]) +"\n stoppingDD: " + str(stoppingDD) + " stoppingDO: " + str(stoppingDO) + 
								" stoppingArrived: " + str(stoppingDArrived) + "\n")
							index+=1
						print(index)				


# (tmpDD, tmpDC, tmpDR, tmpDSC, droneInf) = startSimulation(10,1,1,1,noFault,100)

#lista collisioni per ogni iterazione
collDD, collDC, collDR, collDSC = [], [], [], []

#Lista totale collisioni ostacoli per ogni iterazione
totalCollisionObs = list()

ebsDD, ebsDO, ebsArrived = EBStop(), EBStop(), EBStop()

arrivedDrone = list()

#Otterranno l'output dell'algoritmo di stopping, così da determinare l'interruzione della simulazione
stoppingDD, stoppingDO, stoppingDArrived, stoppingTime = False, False, False, True
index = 0
while((not(stoppingDD and stoppingDO and stoppingDArrived and stoppingTime))):
	(tmpDD, tmpDC, tmpDR, tmpDSC, droneInf) = startSimulation(7,1,1,1,noFault,100)
	collDD.append(tmpDD)
	collDC.append(tmpDC)
	collDR.append(tmpDR)
	collDSC.append(tmpDSC)
	totalCollisionObs.append(collDC[-1] + collDR[-1] + collDSC[-1])
	cont = 0
	for x in droneInf.keys():
		if(droneInf[x][0] > 0.0): cont +=1
	arrivedDrone.append(cont)
	print("droni arrivati all'iterazione " + str(index) + " = "  + str(arrivedDrone[-1])+"\n")
	print("collisioni dd: " ,collDD, "\nobs total: " , totalCollisionObs , "\n")
	if(index > 0):
		stoppingDD = True if(stoppingDD) else ebsDD.find_stop_value(collDD, max(collDD) - min(collDD))
		stoppingDO = True if(stoppingDO) else ebsDO.find_stop_value(totalCollisionObs, max(totalCollisionObs) - min(totalCollisionObs))
		stoppingDArrived = True if(stoppingDArrived) else ebsArrived.find_stop_value(arrivedDrone, max(arrivedDrone) - min(arrivedDrone))
	print("informazioni iterazione num " + str(index) + ": Collisioni tra droni --> " + str(tmpDD) + "\t"+
		" Collisioni DtoO --> " + str(totalCollisionObs[-1]) +"\n stoppingDD: " + str(stoppingDD) + " stoppingDO: " + str(stoppingDO) + 
		" stoppingArrived: " + str(stoppingDArrived) + "\n")
	index+=1
print(index)
