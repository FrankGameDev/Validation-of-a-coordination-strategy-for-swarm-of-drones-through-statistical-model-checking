class System

Drone drone;

Controller ctr;

SetPoint p;

MonitorCollision col;

CollisionAvoidance cad;

PSO pso;

equation 


	for i in 1:K.N loop
		
		//Punto di arrivo
		
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

		//trasferisco la forza dal controller al drone
		connect(drone.Trustx[i], ctr.Trustx[i]);
		connect(drone.Trusty[i], ctr.Trusty[i]);
		connect(drone.Trustz[i], ctr.Trustz[i]);

		//Trasferisco la velocità calcolata dal monitor di flocking al drone
		connect(drone.alignX[i], cad.alignX[i]);
		connect(drone.alignY[i], cad.alignY[i]);
		connect(drone.alignZ[i], cad.alignZ[i]);
		/*print("align x = " + String(cad.alignX[i]) + "\n");
		print("align y = " + String(cad.alignY[i])+ "\n");	
		print("align z = " + String(cad.alignZ[i])+ "\n");
*/
		connect(drone.separateX[i], cad.separateX[i]);
		connect(drone.separateY[i], cad.separateY[i]);
		connect(drone.separateZ[i], cad.separateZ[i]);
		/*print("separate x = " + String(cad.separateX[i])+ "\n");
		print("separate y = " + String(cad.separateY[i])+ "\n");
		print("separate Z = " + String(cad.separateZ[i])+ "\n");
		*/	
		connect(drone.cohesionX[i], cad.cohesionX[i]);
		connect(drone.cohesionY[i], cad.cohesionY[i]);
		connect(drone.cohesionZ[i], cad.cohesionZ[i]);
		/*print("cohesion x = " + String(cad.cohesionX[i])+ "\n");
		print("cohesion y = " + String(cad.cohesionY[i])+ "\n");
		print("cohesion Z = " + String(cad.cohesionZ[i])+ "\n");
*/		
		connect(drone.headingX[i], pso.velocityX[i]);
		connect(drone.headingY[i], pso.velocityY[i]);
		connect(drone.headingZ[i], pso.velocityZ[i]);
		/*print("heading x = " + String(pso.velocityX[i])+ "\n");
		print("heading y = " + String(pso.velocityY[i])+ "\n");
		print("heading Z = " + String(pso.velocityZ[i])+ "\n");
*/

		connect(drone.travelState, ctr.travelState);		

		connect(col.x[i], drone.x[i]);
		connect(col.y[i], drone.y[i]);
		connect(col.z[i], drone.z[i]);

	end for;


end System;
