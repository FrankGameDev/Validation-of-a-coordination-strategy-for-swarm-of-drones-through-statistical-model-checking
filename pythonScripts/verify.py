import os
import sys
import math
import numpy as np
import time
import os.path

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


noFault = [[1, 0, 0, 0],
				[1, 0, 0, 0],
				[1, 0, 0, 0],
				[1, 0, 0, 0]]

faultMatrix = [[0.7, 0.1, 0.1, 0.1],
				[0.6, 0.4, 0, 0],
				[0.5, 0, 0.5, 0],
				[0.7, 0, 0, 0.3]]


    

nDroni = int(omc.sendExpression("getParameterValue(K,\"N\")"))

#Dizionario contenente tempo di arrivo e stato finale di ogni drone
droneInfo = dict.fromkeys([x for x in range(1,nDroni+1)], ()) 
for i in range(1):
	os.system("./System")

	vars = omc.sendExpression("readSimulationResult(\"System_res.mat\",{time})")
	omc.sendExpression("getErrorString()")
	stopTime = int(vars[-1][-1]) #extract the stop time of the simulation

	#extract collision occurance
	tDD = omc.sendExpression("val(colMan.tDD," + str(stopTime) + ", \"System_res.mat\")")
	tDC = omc.sendExpression("val(colMan.tDC," + str(stopTime) + ", \"System_res.mat\")")
	tDR = omc.sendExpression("val(colMan.tDR," + str(stopTime) + ", \"System_res.mat\")")
	tDSC = omc.sendExpression("val(colMan.tDSC," + str(stopTime) + ", \"System_res.mat\")")

	for j in range(1,nDroni+1):
		arrivalTime = omc.sendExpression("val(sucMo.arrivalTime[" + str(j) + "]," + str(stopTime) + ", \"System_res.mat\")")
		droneArrived = omc.sendExpression("val(sucMo.arrived[" + str(j) + "]," + str(stopTime) + ", \"System_res.mat\")")
		droneInfo[j] = (droneArrived, arrivalTime)

	""" 	print("Tempi di arrivo di " + str(j) + ": " + str(arrivalTime)+"\n")
		print("Il drone " + str(j) + " è arrivato? - " + str(droneArrived) + "\n")
	print("tDD: " + str(tDD)+"\n")
	print("tDC: " + str(tDC)+"\n")
	print("tDR: " + str(tDR)+"\n")
	print("tDSC: " + str(tDSC)+"\n") """
	#print(str(omc.sendExpression("val(stud.probPren, 150.0, \"System_res.mat\")")))

	# os.system("rm -f System_res.mat")      # .... to be on the safe side
        
	# print("Monitor value at iteration", i, ": safety =", safety, " liveness = ", liveness, "- with prenotation probability = ", rand1)
	


	
print("Execution time = ", (time.time()-startTime))
