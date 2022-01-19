class System

Drone drone;

Controller ctr;

SetPoint p;

MonitorCollision col;

CollisionAvoidance cad;

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
/*
		connect(ctr.Vx[i],drone.Vx[i]);
		connect(ctr.Vy[i],drone.Vy[i]);
		connect(ctr.Vz[i],drone.Vz[i]);
*/
		//connection tra info drone e modulo collision avoidance		
		connect(cad.Vx[i],drone.Vx[i]);
		connect(cad.Vy[i],drone.Vy[i]);
		connect(cad.Vz[i],drone.Vz[i]);
		connect(cad.x[i], ctr.x[i]);
		connect(cad.y[i], ctr.y[i]);
		connect(cad.z[i], ctr.z[i]);
		connect(cad.collision, col.outCollision);
		
		//passo la velocità del drone (previa verifica del modulo di collission avoidance) al controller
		connect(ctr.Vx[i],cad.newVx[i]);
		connect(ctr.Vy[i],cad.newVy[i]);
		connect(ctr.Vz[i],cad.newVz[i]);

			
		connect(drone.Trustx[i], ctr.Trustx[i]);
		connect(drone.Trusty[i], ctr.Trusty[i]);
		connect(drone.Trustz[i], ctr.Trustz[i]);
		
		connect(col.x[i], drone.x[i]);
		connect(col.y[i], drone.y[i]);
		connect(col.z[i], drone.z[i]);

	end for;


end System;
