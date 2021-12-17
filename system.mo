class System

Drone drone;

Controller ctr;

SetPoint p;

equation

	for i in 1:K.N loop
		
		connect(p.setx[i], ctr.setx[i]);
		connect(p.sety[i], ctr.sety[i]);
		connect(p.setz[i], ctr.setz[i]);

		connect(drone.x[i], ctr.x[i]);
		connect(drone.y[i], ctr.y[i]);
		connect(drone.z[i], ctr.z[i]);

		connect(drone.Vx[i], ctr.Vx[i]);
		connect(drone.Vy[i], ctr.Vy[i]);
		connect(drone.Vz[i], ctr.Vz[i]);

		connect(drone.Trustx[i], ctr.Trustx[i]);
		connect(drone.Trusty[i], ctr.Trusty[i]);
		connect(drone.Trustz[i], ctr.Trustz[i]);
	

	end for;

end System;
