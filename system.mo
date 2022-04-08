class System

Drone drone;

Controller ctr;

SetPoint p;

MonitorCollision col;

CollisionAvoidance cad;

faultSys fault;

PSO pso;

Intruders intruder;

IntrudersPoint intrP;
IntrController intrCtr;

equation 

	//Connection per i droni
	for i in 1:K.N loop
		
		//Connection Controller
		
		connect(ctr.setx[i],p.setx[i]);
		connect(ctr.sety[i],p.sety[i]);
		connect(ctr.setz[i],p.setz[i]);

		//Passo la posizione del drone al controller
		connect(ctr.x[i],drone.x[i]);
		connect(ctr.y[i],drone.y[i]);
		connect(ctr.z[i],drone.z[i]);

		connect(ctr.Vx[i],drone.Vx[i]);
		connect(ctr.Vy[i],drone.Vy[i]);
		connect(ctr.Vz[i],drone.Vz[i]);

		//connection tra info drone e modulo collision avoidance		
		connect(cad.Vx[i],drone.Vx[i]);
		connect(cad.Vy[i],drone.Vy[i]);
		connect(cad.Vz[i],drone.Vz[i]);
		connect(cad.x[i], drone.x[i]);
		connect(cad.y[i], drone.y[i]);
		connect(cad.z[i], drone.z[i]);
		connect(cad.collision, col.outCollision);
		for j in 1:K.N loop
			connect(cad.neighbours[i,j], drone.neighbours[i,j]);
		end for;
		connect(cad.droneState[i], fault.state[i]);

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
		for j in 1:K.N loop
			connect(pso.nearIntr[j], drone.nearIntr[j]);
		end for;
		//trasferisco la forza dal controller al drone
		connect(drone.Trustx[i], ctr.Trustx[i]);
		connect(drone.Trusty[i], ctr.Trusty[i]);
		connect(drone.Trustz[i], ctr.Trustz[i]);

		//Trasferisco la velocità calcolata dal monitor di flocking al drone
		connect(drone.alignX[i], cad.alignX[i]);
		connect(drone.alignY[i], cad.alignY[i]);
		connect(drone.alignZ[i], cad.alignZ[i]);

		connect(drone.separateX[i], cad.separateX[i]);
		connect(drone.separateY[i], cad.separateY[i]);
		connect(drone.separateZ[i], cad.separateZ[i]);
	
		connect(drone.cohesionX[i], cad.cohesionX[i]);
		connect(drone.cohesionY[i], cad.cohesionY[i]);
		connect(drone.cohesionZ[i], cad.cohesionZ[i]);

		connect(drone.headingX[i], pso.velocityX[i]);
		connect(drone.headingY[i], pso.velocityY[i]);
		connect(drone.headingZ[i], pso.velocityZ[i]);
			

		//Connect monitor collisione

		connect(col.x[i], drone.x[i]);
		connect(col.y[i], drone.y[i]);
		connect(col.z[i], drone.z[i]);

		connect(drone.droneState[i], fault.state[i]);
		connect(pso.droneState[i], fault.state[i]);
	

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

		connect(pso.intrX[z], intruder.x[z]);
		connect(pso.intrY[z], intruder.y[z]);
		connect(pso.intrZ[z], intruder.z[z]);

		connect(col.intrX[z], intruder.x[z]);
		connect(col.intrY[z], intruder.y[z]);
		connect(col.intrZ[z], intruder.z[z]);
	end for;

end System;
