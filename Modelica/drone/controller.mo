block Controller

K const;

parameter Real T = 0.01;   //seconds

parameter Real p = -1;
parameter Real kz1 = -(p^2);   
parameter Real kz2 = 2*p;   

parameter Real ky1 = -(p^2);   
parameter Real ky2 = 2*p;  

parameter Real kx1 = -(p^2);   
parameter Real kx2 = 2*p;   

InputReal setx[const.N];
InputReal sety[const.N];
InputReal setz[const.N];

//Posizione e velocità drone
InputReal x[const.N];
InputReal y[const.N];
InputReal z[const.N];
InputReal Vx[const.N];
InputReal Vy[const.N];
InputReal Vz[const.N];

//Destinazione temporanea in caso di imminente collisione
InputReal tmpSetX[const.N];
InputReal tmpSetY[const.N];
InputReal tmpSetZ[const.N];
//Permette di decidere quale punto di arrivo utilizzare nel calcolo del trust
InputBool useTMPDest[const.N];

//Valori restituiti dal flocking module
InputReal alignX[const.N], alignY[const.N], alignZ[const.N];
InputReal cohesionX[const.N], cohesionY[const.N], cohesionZ[const.N];
InputReal separateX[const.N], separateY[const.N], separateZ[const.N];
//direzione da prendere in aggiunta alla velocità, generata dal PSO
InputReal headingX[const.N], headingY[const.N], headingZ[const.N];

//Stato del drone, fornito dal faultSys
InputInt droneState[const.N];
InputBool droneDead[const.N];

//Scarica batteria dovuta al modulo di sensoristica + manovra (ricerca vicini e spostamento) e al modulo di comunicazione
InputReal batterySensDischarge[const.N], batteryPSODischarge[const.N];

//Forza
OutputReal Trustx[const.N];
OutputReal Trusty[const.N];
OutputReal Trustz[const.N];

Real destX,destY,destZ;
	
Real tmpFx,tmpFy,tmpFz;

Real battery[const.N];

parameter Real cohesionWeight = 1;	
parameter Real alignWeight = 6;
parameter Real separateWeight = 6;
parameter Real headingWeight = 5;
parameter Real vWeight = 5;

algorithm
	for i in 1:const.N loop
		battery[i] := batterySensDischarge[i] - batteryPSODischarge[i];
		if(droneState[i] <> 3 and battery[i] > 0 and (not droneDead[i])) then
			if(useTMPDest[i]) then
				destX := tmpSetX[i];
				destY := tmpSetY[i];
				destZ := tmpSetZ[i];

				Trustx[i] := (kx1*(x[i] - destX) + kx2*Vx[i]);
				Trusty[i] := (ky1*(y[i] - destY) + ky2*Vy[i]);
				Trustz[i] := (const.m*(const.g + kz1*(z[i] - destZ) + kz2*Vz[i]))- const.m*const.g;
			else
				destX := setx[i];
				destY := sety[i];
				destZ := setz[i];

				tmpFx := const.m*(kx1*(x[i] - destX) + kx2*Vx[i]);
				tmpFy := const.m*(ky1*(y[i] - destY) + ky2*Vy[i]);
				tmpFz := const.m*(const.g + kz1*(z[i] - destZ) + kz2*Vz[i]) - const.m*const.g;

				Trustx[i] := (tmpFx/const.m)*vWeight + (alignX[i]*alignWeight + cohesionX[i]*cohesionWeight + separateX[i]*separateWeight + headingX[i]*headingWeight); 
				Trusty[i] := (tmpFy/const.m)*vWeight + (alignY[i]*alignWeight + cohesionY[i]*cohesionWeight + separateY[i]*separateWeight + headingY[i]*headingWeight);
				Trustz[i] := (tmpFz/const.m)*vWeight + (alignZ[i]*alignWeight + cohesionZ[i]*cohesionWeight + separateZ[i]*separateWeight + headingZ[i]*headingWeight);		
			end if;
		else
			Trustx[i] := if(z[i] > 5) then -(const.m * const.g) else 0;
			Trusty[i] := if(z[i] > 5) then -(const.m * const.g) else 0;
			Trustz[i] := if(z[i] > 5) then -(const.m * const.g) else 0;
		end if;
		//Velocity cap
		(Trustx[i],Trusty[i],Trustz[i]) := velocityCap(Trustx[i],Trusty[i],Trustz[i],const.maxSpeed);	

		
	end for;


end Controller;



