class System

Drone drone;

Controller ctr;

SetPoint p;

MonitorSuccess sucMo;

CollisionManagement colMan;

flockingModule flock;

CollisionAvoidance cad;

faultSys fault;

PSO pso;

Intruders intruder;
IntrudersPoint intrP;
IntrController intrCtr;

Rocket rocket;
RocketPointer rockP;
RockController rockCtr;

StaticObs statObs;

equation 

	//Connection per i droni
	for i in 1:K.N loop
		
		connect(p.startX[i], drone.startPos[i,1]);
		connect(p.startY[i], drone.startPos[i,2]);
		connect(p.startZ[i], drone.startPos[i,3]);
		connect(p.battery[i], drone.actualCapacity[i]);

		//Connection Controller
		connect(ctr.setx[i],p.setx[i]);
		connect(ctr.sety[i],p.sety[i]);
		connect(ctr.setz[i],p.setz[i]);
		connect(ctr.x[i],drone.x[i]);
		connect(ctr.y[i],drone.y[i]);
		connect(ctr.z[i],drone.z[i]);
		connect(ctr.Vx[i],drone.Vx[i]);
		connect(ctr.Vy[i],drone.Vy[i]);
		connect(ctr.Vz[i],drone.Vz[i]);
		connect(ctr.tmpSetX[i], cad.tmpDestX[i]);
		connect(ctr.tmpSetY[i], cad.tmpDestY[i]);
		connect(ctr.tmpSetZ[i], cad.tmpDestZ[i]);
		connect(ctr.useTMPDest[i], cad.useTMPDest[i]);
		//Velocità calcolate dal modulo di flocking
		connect(ctr.alignX[i], flock.alignX[i]);
		connect(ctr.alignY[i], flock.alignY[i]);
		connect(ctr.alignZ[i], flock.alignZ[i]);
		connect(ctr.separateX[i], flock.separateX[i]);
		connect(ctr.separateY[i], flock.separateY[i]);
		connect(ctr.separateZ[i], flock.separateZ[i]);
		connect(ctr.cohesionX[i], flock.cohesionX[i]);
		connect(ctr.cohesionY[i], flock.cohesionY[i]);
		connect(ctr.cohesionZ[i], flock.cohesionZ[i]);
		//Trasferisco l'heading vector dal pso al drone
		connect(ctr.headingX[i], pso.velocityX[i]);
		connect(ctr.headingY[i], pso.velocityY[i]);
		connect(ctr.headingZ[i], pso.velocityZ[i]);
		//Stato di fault del drone + Stato drone
		connect(ctr.droneState[i],fault.state[i]);
		connect(ctr.droneDead[i],colMan.droneDead[i]);
		//Valori di scarica della batteria
		connect(ctr.batterySensDischarge[i], drone.actualCapacity[i]);
		connect(ctr.batteryPSODischarge[i], pso.batteryDischarge[i]);


		//connection tra info drone e modulo flocking		
		connect(flock.Vx[i],drone.Vx[i]);
		connect(flock.Vy[i],drone.Vy[i]);
		connect(flock.Vz[i],drone.Vz[i]);
		connect(flock.x[i], drone.x[i]);
		connect(flock.y[i], drone.y[i]);
		connect(flock.z[i], drone.z[i]);
		for j in 1:K.N loop
			connect(flock.neighbours[i,j], drone.neighbours[i,j]);
		end for;
		connect(flock.droneState[i], fault.state[i]);
		connect(flock.battery[i], drone.actualCapacity[i]);
		connect(flock.droneDead[i], colMan.droneDead[i]);

		//Connection tra drone e modulo collision avoidance
		connect(cad.x[i], drone.x[i]);
		connect(cad.y[i], drone.y[i]);
		connect(cad.z[i], drone.z[i]);
		connect(cad.destX[i],p.setx[i]);
		connect(cad.destY[i],p.sety[i]);
		connect(cad.destZ[i],p.setz[i]);
		connect(cad.droneState[i], fault.state[i]);
		connect(cad.nearIntr[i], drone.nearIntr[i]);
		connect(cad.nearMissile[i], drone.nearMissile[i]);
		connect(cad.nearStatObs[i], drone.nearStatObs[i]);
		connect(cad.battery[i], drone.actualCapacity[i]);
		connect(cad.droneDead[i], colMan.droneDead[i]);


		//connection tra pso e valori drone + posizione di arrivo
		connect(pso.Vx[i],drone.Vx[i]);
		connect(pso.Vy[i],drone.Vy[i]);
		connect(pso.Vz[i],drone.Vz[i]);
		connect(pso.x[i], drone.x[i]);
		connect(pso.y[i], drone.y[i]);
		connect(pso.z[i], drone.z[i]);
		connect(pso.destX[i],p.setx[i]);
		connect(pso.destY[i],p.sety[i]);
		connect(pso.destZ[i],p.setz[i]);
		for j in 1:K.N loop
			connect(pso.neighbours[i,j], drone.neighbours[i,j]);
		end for;
		connect(pso.nearIntr[i], drone.nearIntr[i]);
		connect(pso.nearMissile[i], drone.nearMissile[i]);
		connect(pso.droneState[i], fault.state[i]);
		connect(pso.battery[i], drone.actualCapacity[i]);
		connect(pso.droneDead[i], colMan.droneDead[i]);
		connect(pso.nearStatObs[i], drone.nearStatObs[i]);


		//trasferisco la forza dal controller al drone
		connect(drone.destX[i], p.setx[i]);
		connect(drone.destY[i], p.sety[i]);
		connect(drone.destZ[i], p.setz[i]);
		connect(drone.Trustx[i], ctr.Trustx[i]);
		connect(drone.Trusty[i], ctr.Trusty[i]);
		connect(drone.Trustz[i], ctr.Trustz[i]);
		//trasferisco la scarica della batteria dovuta al modulo di comunicazione
		connect(drone.commDischarge[i], pso.batteryDischarge[i]);
		//Asserisco se il drone sta usando la nuova destinazione identificata dal monitor di collision avoidance
		connect(drone.useTMPDest[i], cad.useTMPDest[i]);
		connect(drone.droneState[i], fault.state[i]);
		connect(drone.droneDead[i], colMan.droneDead[i]);

		//Connect collision Management
		connect(colMan.x[i], drone.x[i]);
		connect(colMan.y[i], drone.y[i]);
		connect(colMan.z[i], drone.z[i]);

		//connect monitor success
		connect(sucMo.x[i], drone.x[i]);
		connect(sucMo.y[i], drone.y[i]);
		connect(sucMo.z[i], drone.z[i]);
		connect(sucMo.destX[i], p.setx[i]);
		connect(sucMo.destY[i], p.sety[i]);
		connect(sucMo.destZ[i], p.setz[i]);
		connect(sucMo.batterySensDischarge[i], drone.actualCapacity[i]);
		connect(sucMo.batteryPSODischarge[i], pso.batteryDischarge[i]);
		connect(sucMo.droneDead[i], colMan.droneDead[i]);

		//connection pointer missili con posizione droni
		connect(rockP.droneX[i],drone.x[i]);
		connect(rockP.droneY[i],drone.y[i]);
		connect(rockP.droneZ[i],drone.z[i]);

	end for;
	
	//Connection per gli intruders
	for z in 1:K.nIntr loop
		connect(intrCtr.setx[z],intrP.setx[z]);
		connect(intrCtr.sety[z],intrP.sety[z]);
		connect(intrCtr.setz[z],intrP.setz[z]);

		connect(intrCtr.x[z],intruder.x[z]);
		connect(intrCtr.y[z],intruder.y[z]);
		connect(intrCtr.z[z],intruder.z[z]);

		connect(intrCtr.Vx[z],intruder.Vx[z]);
		connect(intrCtr.Vy[z],intruder.Vy[z]);
		connect(intrCtr.Vz[z],intruder.Vz[z]);

		connect(intruder.Trustx[z], intrCtr.Trustx[z]);
		connect(intruder.Trusty[z], intrCtr.Trusty[z]);
		connect(intruder.Trustz[z], intrCtr.Trustz[z]);

		connect(drone.intrX[z], intruder.x[z]);
		connect(drone.intrY[z], intruder.y[z]);
		connect(drone.intrZ[z], intruder.z[z]);
		connect(drone.intrDead[z], colMan.intrDead[z]);

		connect(pso.intrX[z], intruder.x[z]);
		connect(pso.intrY[z], intruder.y[z]);
		connect(pso.intrZ[z], intruder.z[z]);
		connect(pso.intrDead[z], colMan.intrDead[z]);

		connect(colMan.intrX[z], intruder.x[z]);
		connect(colMan.intrY[z], intruder.y[z]);
		connect(colMan.intrZ[z], intruder.z[z]);

		connect(cad.intrX[z], intruder.x[z]);
		connect(cad.intrY[z], intruder.y[z]);
		connect(cad.intrZ[z], intruder.z[z]);	
		connect(cad.intrDead[z], colMan.intrDead[z]);

	end for;

	//Connection missili
	for q in 1:K.nRocket loop

		connect(rockP.x[q],rocket.x[q]);
		connect(rockP.y[q],rocket.y[q]);
		connect(rockP.z[q],rocket.z[q]);
		
		connect(rockCtr.setx[q],rockP.setx[q]);
		connect(rockCtr.sety[q],rockP.sety[q]);
		connect(rockCtr.setz[q],rockP.setz[q]);

		connect(rockCtr.x[q],rocket.x[q]);
		connect(rockCtr.y[q],rocket.y[q]);
		connect(rockCtr.z[q],rocket.z[q]);
		connect(rockCtr.Vx[q],rocket.Vx[q]);
		connect(rockCtr.Vy[q],rocket.Vy[q]);
		connect(rockCtr.Vz[q],rocket.Vz[q]);
		connect(rockCtr.targetReached[q], rockP.targetReached[q]);

		connect(rocket.Trustx[q], rockCtr.Trustx[q]);
		connect(rocket.Trusty[q], rockCtr.Trusty[q]);
		connect(rocket.Trustz[q], rockCtr.Trustz[q]);

		connect(drone.missX[q], rocket.x[q]);
		connect(drone.missY[q], rocket.y[q]);
		connect(drone.missZ[q], rocket.z[q]);
		connect(drone.missDead[q], colMan.missDead[q]);

		connect(pso.missX[q], rocket.x[q]);
		connect(pso.missY[q], rocket.y[q]);
		connect(pso.missZ[q], rocket.z[q]);
		connect(pso.missDead[q], colMan.missDead[q]);

		connect(colMan.missX[q], rocket.x[q]);
		connect(colMan.missY[q], rocket.y[q]);
		connect(colMan.missZ[q], rocket.z[q]);

		connect(cad.missX[q], rocket.x[q]);
		connect(cad.missY[q], rocket.y[q]);
		connect(cad.missZ[q], rocket.z[q]);
		connect(cad.missDead[q], colMan.missDead[q]);

	end for;

	for l in 1:K.nStatObs loop
		connect(drone.statX[l], statObs.x[l]);
		connect(drone.statY[l], statObs.y[l]);
		connect(drone.statZ[l], statObs.z[l]);

		connect(pso.statX[l], statObs.x[l]);
		connect(pso.statY[l], statObs.y[l]);
		connect(pso.statZ[l], statObs.z[l]);

		connect(cad.statX[l], statObs.x[l]);
		connect(cad.statY[l], statObs.y[l]);
		connect(cad.statZ[l], statObs.z[l]);

		connect(colMan.statX[l], statObs.x[l]);
		connect(colMan.statY[l], statObs.y[l]);
		connect(colMan.statZ[l], statObs.z[l]);
	end for;

end System;