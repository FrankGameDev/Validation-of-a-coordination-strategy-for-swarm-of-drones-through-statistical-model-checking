block Controller

parameter Real T = 0.01;   //seconds

parameter Real p = -1;
parameter Real kz1 = -(p^2);   
parameter Real kz2 = 2*p;   

parameter Real ky1 = -(p^2);   
parameter Real ky2 = 2*p;  

parameter Real kx1 = -(p^2);   
parameter Real kx2 = 2*p;   

InputReal setx[K.N];
InputReal sety[K.N];
InputReal setz[K.N];

//Posizione e velocità drone
InputReal x[K.N];
InputReal y[K.N];
InputReal z[K.N];
InputReal Vx[K.N];
InputReal Vy[K.N];
InputReal Vz[K.N];

//Destinazione temporanea in caso di imminente collisione
InputReal tmpSetX[K.N];
InputReal tmpSetY[K.N];
InputReal tmpSetZ[K.N];
//Permette di decidere quale punto di arrivo utilizzare nel calcolo del trust
InputBool useTMPDest[K.N];

//Valori restituiti dal flocking module
InputReal alignX[K.N], alignY[K.N], alignZ[K.N];
InputReal cohesionX[K.N], cohesionY[K.N], cohesionZ[K.N];
InputReal separateX[K.N], separateY[K.N], separateZ[K.N];
//direzione da prendere in aggiunta alla velocità, generata dal PSO
InputReal headingX[K.N], headingY[K.N], headingZ[K.N];

//Stato del drone, fornito dal faultSys
InputInt droneState[K.N];
InputBool droneDead[K.N];

//Scarica batteria dovuta al modulo di sensoristica + manovra (ricerca vicini e spostamento) e al modulo di comunicazione
InputReal batterySensDischarge[K.N], batteryPSODischarge[K.N];

//Forza
OutputReal Trustx[K.N];
OutputReal Trusty[K.N];
OutputReal Trustz[K.N];

Real destX,destY,destZ;
	
Real tmpFx,tmpFy,tmpFz;

Real battery[K.N];

parameter Real cohesionWeight = 1;	
parameter Real alignWeight = 6;
parameter Real separateWeight = 6;
parameter Real headingWeight = 10;
parameter Real vWeight = 5;

algorithm
	for i in 1:K.N loop
		battery[i] := batterySensDischarge[i] - batteryPSODischarge[i];
		if(droneState[i] <> 3 and battery[i] > 0 and (not droneDead[i])) then
			if(useTMPDest[i]) then
				destX := tmpSetX[i];
				destY := tmpSetY[i];
				destZ := tmpSetZ[i];

				Trustx[i] := (kx1*(x[i] - destX) + kx2*Vx[i]);
				Trusty[i] := (ky1*(y[i] - destY) + ky2*Vy[i]);
				Trustz[i] := (K.m*(K.g + kz1*(z[i] - destZ) + kz2*Vz[i]))- K.m*K.g;
			else
				destX := setx[i];
				destY := sety[i];
				destZ := setz[i];

				tmpFx := K.m*(kx1*(x[i] - destX) + kx2*Vx[i]);
				tmpFy := K.m*(ky1*(y[i] - destY) + ky2*Vy[i]);
				tmpFz := K.m*(K.g + kz1*(z[i] - destZ) + kz2*Vz[i]) - K.m*K.g;

				Trustx[i] := (tmpFx/K.m)*vWeight + (alignX[i]*alignWeight + cohesionX[i]*cohesionWeight + separateX[i]*separateWeight + headingX[i]*headingWeight); 
				Trusty[i] := (tmpFy/K.m)*vWeight + (alignY[i]*alignWeight + cohesionY[i]*cohesionWeight + separateY[i]*separateWeight + headingY[i]*headingWeight);
				Trustz[i] := (tmpFz/K.m)*vWeight + (alignZ[i]*alignWeight + cohesionZ[i]*cohesionWeight + separateZ[i]*separateWeight + headingZ[i]*headingWeight);		
			end if;
		else
			Trustx[i] := 0;
			Trusty[i] := 0;
			Trustz[i] := if(z[i] > 5) then -(K.m * K.g) else 0;
		end if;
		//Velocity cap
		(Trustx[i],Trusty[i],Trustz[i]) := velocityCap(Trustx[i],Trusty[i],Trustz[i],K.maxSpeed);	

		
	end for;


end Controller;



