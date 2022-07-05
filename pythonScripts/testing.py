import os, psutil
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

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/connectors.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/constants.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/randgen.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/extFunction.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/drone/drone.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/Monitors/MonitorSuccess.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/Monitors/MonitorCollision.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/algorithms/flockingModule.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/algorithms/psoController.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/algorithms/collisionAvoidance.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/drone/controller.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/drone/setPoint.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/faultSystem.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/intruders/intruders.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/intruders/intrudersController.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/intruders/intrudersPoint.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/intruders/rockets.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/intruders/rocketController.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/intruders/rocketPoint.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/intruders/staticObs.mo\")")
omc.sendExpression("getErrorString()")

omc.sendExpression("loadFile(\"/home/francesco/Scrivania/Drones/Modelica/system.mo\")")
omc.sendExpression("getErrorString()")

startTime = time.time()
noFault = "{{1, 0, 0, 0},{1, 0, 0, 0},{1, 0, 0, 0},{1, 0, 0, 0}}"

#Scrive il file contenente le informazioni sotto forma di json
def writeData(name,collDD, collDO, arrived, dTime, droneF = {}):
	if(droneF == {}): diz = {"collDD": collDD, "collDO": collDO, "arrived": arrived, "arrivalTime":dTime}
	else: diz = {"collDD": collDD, "collDO": collDO, "arrived": arrived, "arrivalTime":dTime, "droneFault":droneF}
	with open("/home/francesco/Scrivania/Drones/Simulation_Data/SimulationData_" + str(name) + ".json", "wt") as f:
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

	faultSequence = [[] for _ in range(ndrone)]

	#extract collision occurance
	tDD = omc.sendExpression("val(colMan.tDD," + str(stopTime) + ", \"System_res.mat\")") #Collisioni droni
	tDC = omc.sendExpression("val(colMan.tDC," + str(stopTime) + ", \"System_res.mat\")") #Collisioni droni-intrusi
	tDR = omc.sendExpression("val(colMan.tDR," + str(stopTime) + ", \"System_res.mat\")") #Collisioni droni-missili
	tDSC = omc.sendExpression("val(colMan.tDSC," + str(stopTime) + ", \"System_res.mat\")") #Collisioni droni-ostacoli statici
	for j in range(1,ndrone+1):
		#extract arrival time and number of drone arrived at destination
		arrivalTime = omc.sendExpression("val(sucMo.arrivalTime[" + str(j) + "]," + str(stopTime) + ", \"System_res.mat\")")
		droneArrived = omc.sendExpression("val(sucMo.arrived[" + str(j) + "]," + str(stopTime) + ", \"System_res.mat\")")
		droneInfo[j] = (droneArrived, arrivalTime)
		
		#extract fault info
		for t in range(stopTime+1):
			f_at_t = omc.sendExpression("val(fault.state[" + str(j) + "]," + str(t) + ", \"System_res.mat\")")
			faultSequence[j-1].append(f_at_t)
	droneFault = dict.fromkeys([x for x in range(1,ndrone+1)], [])
	for d in range(len(droneFault.keys())):
		droneFault[d+1] = faultSequence[d]
	# print(tDD, tDC, tDR, tDSC)

	os.system("rm -f System_res.mat")      # .... to be on the safe side
	os.system("rm -f newValues.txt")      # .... to be on the safe side
	return (tDD, tDC, tDR, tDSC, droneInfo, droneFault) 

#Esegue le simulazioni di uno scenario attraverso EBGStop
def simulate_with_ebgstop(d, zone, fault, nomeF):

	intruders = 10
	missile = 10
	staticObs = 30

	#Azzero la percentuale di utilizzo della ram totale
	total_mem_usage = 0

	startTime = time.time()

	print("simulazione con: (" + str(d) +" droni, area di volo: "+ str(zone) +")\n")
	#lista collisioni per ogni iterazione 
	collDD, collDC, collDR, collDSC = [], [], [], []

	#Lista totale collisioni ostacoli per ogni iterazione
	totalCollisionObs = list()

	ebsDD, ebsDO, ebsArrived, ebsTime, ebsFault = EBStop(), EBStop(), EBStop(), EBStop(), EBStop()

	arrivedDrone, arrivalTime = list(), list()
	tmptime = [[] for _ in range(d)]

	droneFaultOverTime = dict() 

	faultAverage = []

	#Otterranno l'output dell'algoritmo di stopping, così da determinare l'interruzione della simulazione
	stoppingDD, stoppingDO, stoppingDArrived, stoppingTime, stoppingFault = False, False, False, False, False
	valDD, valDO, valArrived, valTime, valFault = 0,0,0,0,0 
	index = 0
	process = psutil.Process(os.getpid())
	mem = process.memory_percent()
	mem_mb = process.memory_info().rss / 1024 ** 2
	print("ram%: ", mem, "ram mb: ", mem_mb)
	parameterSweep(d,intruders,missile,staticObs,fault,zone)
	while((not(stoppingDD and stoppingDO and stoppingDArrived and stoppingTime and stoppingFault))):
		(tmpDD, tmpDC, tmpDR, tmpDSC, droneInf, droneFault) = startSimulation(zone)	
		collDD.append(tmpDD)
		collDC.append(tmpDC)
		collDR.append(tmpDR)
		collDSC.append(tmpDSC)
		totalCollisionObs.append(collDC[-1] + collDR[-1] + collDSC[-1])
		# print(totalCollisionObs)
		cont = 0
		#tmpList salva temporaneamente i tempi di arrivo dei droni, così da farne la media per l'iterazione corrente
		tmpList = list()
		droneIndex = 1
		print(index)
		#Salvo numero di droni arrivati
		for x in droneInf.keys():
			if(droneInf[x][0] > 0.0): #se il drone è arrivato
				cont += 1
			if(droneInf[x][1] > -1.0):
				tmpList.append(droneInf[x][1])
				tmptime[droneIndex-1].append(droneInf[x][1])
			droneIndex += 1
		arrivedDrone.append(cont)
		#Memorizzo tempo di arrivo medio
		if(len(tmpList) >  0): arrivalTime.append(np.sum(tmpList)/len(tmpList))
		#calcolo media fault presente nella i-esima simulazione, omettendo il caso di fault=1(ossia quando non sono presenti)
		sommaFault = 0
		for dd in droneFault:
			for faults in droneFault[dd]:
				if(faults != 1.0): sommaFault += 1
		faultAverage.append(sommaFault)
		#Eseguo i vari EBGStop
		if(index > 0):
			stoppingDD,tmpvalDD = ebsDD.find_stop_value(collDD, max(collDD) - min(collDD))
			print("stoppingDD:",stoppingDD, valDD)
			stoppingDO,tmpDO =  ebsDO.find_stop_value(totalCollisionObs, max(totalCollisionObs) - min(totalCollisionObs))
			print("stoppingDO:",stoppingDO,valDO)
			stoppingDArrived, tmpArrived = ebsArrived.find_stop_value(arrivedDrone, max(arrivedDrone) - min(arrivedDrone))
			print("stoppingArrived:",stoppingDArrived, valArrived)
			stoppingTime, tmpvalTime = ebsTime.find_stop_value(arrivalTime, max(arrivalTime) - min(arrivalTime))
			print("stoppingTime:", stoppingTime, valTime)
			stoppingFault, tmpFault = ebsFault.find_stop_value(faultAverage, max(faultAverage) - min(faultAverage))
			print("stoppingFault:",stoppingFault, valFault)
			valDD = tmpvalDD if(not math.isnan(tmpvalDD)) else valDD
			valDO = tmpDO if(not math.isnan(tmpDO)) else valDO
			valArrived = tmpArrived if(not math.isnan(tmpArrived)) else valArrived
			valTime = tmpvalTime if(not math.isnan(tmpvalTime)) else valTime
			valFault = tmpFault if(not math.isnan(tmpFault)) else valFault
		droneFaultOverTime.update({index:droneFault})
		index+=1
	print("Tempo di esecuzione = ", (time.time() - startTime))
	#completo il dizionario contenente i tempi di arrivo dei droni
	droneArrivalTime = dict.fromkeys([i for i in range(1,d+1)],[])
	for i in range(len(droneArrivalTime.keys())):
		droneArrivalTime[i+1] = tmptime[i]

	# Scrivo i risultati all'interno di un file json
	writeData(str(d) + "_" + str(intruders) + "_" +  str(missile) + "_" + str(staticObs) + "_" + str(nomeF) +
			"_" + str(zone), collDD, totalCollisionObs, arrivedDrone, droneArrivalTime, droneFaultOverTime)

	# Salvo i valori attesi restituiti da ebgstop
	writeData(str(d) + "_" + str(intruders) + "_" +  str(missile) + "_" + str(staticObs) + "_" + str(nomeF) +
			"_" + str(zone) + "_EBSVALUES", valDD, valDO, valArrived, valTime, valFault)

	# Salvo in un file il tempo di simulazione, il numero di simulazioni e il percentuale di uso della ram per lo scenario simulato
	with open("/home/francesco/Scrivania/Drones/Simulation_Data/LogEBS.txt", "a") as f:
		f.write("Simulazione " +str(d) + "_" + str(intruders) + "_" +  str(missile) + "_" + str(staticObs) + "_" + str(nomeF) +
			"_" + str(zone)+ ". Tempo di esecuzione: " + str(time.time()-startTime) +"; RAM in mb: "+ str(mem_mb) +"; % RAM media: "+ str(mem) +"; Iterazioni EBS: " + str(index) + ";\n")
		f.flush()
		os.fsync(f)

#Esegue simulazioni del sistema per ogni scenario
def get_simulation_data():
	# with open("Simulation_Data/LogEBS_" + str(time.time()) + ".txt", "wt") as f:
	# 	f.write("Tempo computazionale per ogni scenario attraverso EBS\n")
	# 	f.flush()
	# 	os.fsync(f)

	drones = [20]
	intruders = 10
	missile = 10
	staticObs = 8
	flyZone = [100,150,200]
	noFault = "{{1, 0, 0, 0},{1, 0, 0, 0},{1, 0, 0, 0},{1, 0, 0, 0}}"
	faultMatrix = "{{0.7, 0.1, 0.1, 0.1},{0.6, 0.4, 0, 0},{0.5, 0, 0.5, 0},{0.7, 0, 0, 0.3}}"

	#Variabile temporanea per assegnazione nome file
	nf = 0
	nomeF = "no"
	for fault in ([faultMatrix]):
		print(fault)
		if(nf>0): nomeF = "si"
		for d in drones:
			for zone in flyZone:
				simulate_with_ebgstop(d,zone,fault,nomeF)
		nf+=1

get_simulation_data()
with open("/home/francesco/Scrivania/Drones/Simulation_Data/LogEBS.txt", "a") as f:
	f.write("Tempo di esecuzione totale = " + str(time.time()-startTime)+";\n")
	f.flush()
	os.fsync(f)
print("Execution time = ", (time.time()-startTime))
