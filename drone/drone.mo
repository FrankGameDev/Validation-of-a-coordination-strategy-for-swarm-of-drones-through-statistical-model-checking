block Drone
	
	parameter Real T = 0.5;

//PARAMETRI BATTERIA

	//capacità massima(mAh)	
	parameter Real capacity = 5000;

//Parametri volo

	//Destinazione droni
	InputReal destX[K.N];
	InputReal destY[K.N];
	InputReal destZ[K.N];

	//forza di movimento del drone
	InputReal Trustx[K.N];
	InputReal Trusty[K.N];
	InputReal Trustz[K.N];	

	//Valori restituiti dal collision avoidance module
	InputReal alignX[K.N];
	InputReal alignY[K.N];
	InputReal alignZ[K.N];

	InputReal cohesionX[K.N];
	InputReal cohesionY[K.N];
	InputReal cohesionZ[K.N];
	
	InputReal separateX[K.N];
	InputReal separateY[K.N];
	InputReal separateZ[K.N];
	
	//direzione da prendere in aggiunta alla velocità, generata dal PSO

	InputReal headingX[K.N];
	InputReal headingY[K.N];
	InputReal headingZ[K.N];

	//Campi di fault

	//Stato del drone. 1 = funzionante; 2 = errore sensoristica; 3 = errore di manovra; 4 = errore comunicazioni;
	InputInt droneState[K.N];

	//Permette di sapere se droni,ostacoli o missili sono collisi
	InputBool droneDead[K.N];
    InputBool intrDead[K.nIntr];
    InputBool missDead[K.nRocket];
	
	//Campi posizione intrusi

	InputReal intrX[K.nIntr];
	InputReal intrY[K.nIntr];
	InputReal intrZ[K.nIntr];

	//Posizione missili
	InputReal missX[K.nRocket];
	InputReal missY[K.nRocket];
	InputReal missZ[K.nRocket];


	//Scarica della batteria dovuta all'utilizzo del modulo di comunicazione
	InputReal commDischarge[K.N];

	//Valore che permette di definire se il modulo di collision avoidance ha determinato
	//una nuova destinazione per schivare gli ostacoli
	InputBool useTMPDest[K.N];

	//Posizione sull'asse x
	OutputReal x[K.N];

	//Posizione sull'asse y
	OutputReal y[K.N];
	
	//Posizione sull'asse z
	OutputReal z[K.N];


	//Forza su x
	OutputReal Fx[K.N];
	
	//Forza su y
	OutputReal Fy[K.N];

	//Forza su z
	OutputReal Fz[K.N];
	
	//Velocità su x
	OutputReal Vx[K.N];

	//Velocità su y
	OutputReal Vy[K.N];

	//Velocità su z
	OutputReal Vz[K.N];

	//Array contenente informazioni sui vicini al drone i-esimo
	OutputBool neighbours[K.N,K.N];

	//Vettore contenente informazioni sugli intrusi vicini
	OutputBool nearIntr[K.N, K.nIntr];

	//Vettore contenente informazioni sui missili vicini
	OutputBool nearMissile[K.N, K.nRocket];

	//capacità della batteria di ogni drone
	OutputReal actualCapacity[K.N]; 	

	//Start position of drones
	OutputReal startPos[K.N,3];

	Real tmpBatt;
	
//Parametri sull'applicazione degli algoritmi di flocking e Pathfinding

//La somma dei vari pesi degli attributi deve essere uguale a 1.
	parameter Real cohesionWeight = 0.5;	
	parameter Real alignWeight = 1;
	parameter Real separateWeight = 3;
	parameter Real headingWeight = 1.5;
	parameter Real vWeight = 5;

initial equation
	
	for i in 1:K.N loop
		x[i] = 0;
		y[i] = i*K.dDistance;
		z[i]= 5+i*K.dDistance;
	end for;


	for i in 1:K.N loop
		Vx[i] = 0;
		Vy[i] = 0;
		Vz[i] = 0;
	end for;	

	actualCapacity = fill(capacity,K.N);
	tmpBatt = actualCapacity[1];
equation
	/*
	* f = m * a --> a =f/m
	* f= m der(v)
	* v = der(x)
	*/
	
	
	for i in 1:K.N loop 
		//Se non ci sono fault di manovra e la batteria ha ancora carica residua, leggo la trust del controller
		if(droneState[i] <> 3 and actualCapacity[i] > 0 and (not droneDead[i])) then	
			Fx[i] = Trustx[i];	
			Fy[i] = Trusty[i];
			Fz[i] = Trustz[i] - K.m * K.g;
		else
			Fx[i] = 0;	
			Fy[i] = 0;
			Fz[i] = -(K.m * K.g);
		end if;	
	end for;


	for i in 1:K.N loop
		if(droneDead[i]) then
			der(Vx[i]) = 0;
			der(Vy[i]) = 0;
			der(Vz[i]) = 0;
		elseif(useTMPDest[i] or (droneState[i] == 3 and actualCapacity[i] <= 0)) then
			der(Vx[i]) = (Fx[i]/K.m);
			der(Vy[i]) = (Fy[i]/K.m);
			der(Vz[i]) = (Fz[i]/K.m);
		else
			der(Vx[i]) = (Fx[i]/K.m)*vWeight + (alignX[i]*alignWeight + cohesionX[i]*cohesionWeight + separateX[i]*separateWeight + headingX[i]*headingWeight); 
			der(Vy[i]) = (Fy[i]/K.m)*vWeight + (alignY[i]*alignWeight + cohesionY[i]*cohesionWeight + separateY[i]*separateWeight + headingY[i]*headingWeight);
			der(Vz[i]) = (Fz[i]/K.m)*vWeight + (alignZ[i]*alignWeight + cohesionZ[i]*cohesionWeight + separateZ[i]*separateWeight + headingZ[i]*headingWeight);
		end if;
		der(x[i]) = Vx[i];
		der(y[i]) = Vy[i];
		der(z[i]) = Vz[i];
	end for;



algorithm

when initial() then
	for i in 1:K.N loop
		neighbours[i] := fill(true, K.N);
		startPos[i,1] := x[i];
		startPos[i,2] := y[i];
		startPos[i,3] := z[i];
		nearIntr := fill(false, K.N, K.nIntr);
		nearMissile	:= fill(false, K.N, K.nRocket);
	end for;
end when;

when sample(0,T) then

	for i in 1:K.N loop
		if(actualCapacity[i] > 0 and (not droneDead[i])) then
			tmpBatt := actualCapacity[i];
			actualCapacity[i] := batteryMonitor(tmpBatt,1);
			//Se i sensori del drone non funzionano allora non trova gli oggetti vicini
			if(droneState[i] <> 2) then
				(neighbours[i], nearIntr[i], nearMissile[i]) := findNearObject(x[i], y[i], z[i], x,y,z, intrX, intrY, intrZ, missX, missY, missZ);
				tmpBatt := actualCapacity[i];
				actualCapacity[i] := batteryMonitor(tmpBatt,2);
			else
				neighbours[i] := fill(false, K.N);	
				nearIntr[i] := fill(false, K.nIntr);
				nearMissile[i] := fill(false,K.nRocket);
			end if;	
		end if;
	end for;

	//(neighbours, nearIntr, nearMissile) := seeNearObject(x, y, z, destX, destY,destZ, intrX, intrY, intrZ, missX, missY, missZ, neighbours, nearIntr, nearMissile);

	
end when;
	
	
end Drone;


function batteryMonitor

	InputReal battery;
	
	//Scarica della batteria
	InputReal dischargeRate;
	
	//Discharge rate dovuta al modulo di comunicazione
	//InputReal commDischarge;
		
	OutputReal outBattery;

algorithm
	
	if(battery <= 0) then 
		outBattery := 0;
	else
		outBattery := battery - dischargeRate;
	end if;
end batteryMonitor;

function returnedHome

	InputReal x,y,z;
	InputReal startX,startY,startZ;

	OutputBool res;

algorithm
	if(x == startX and y == startY and z == startZ) then
		res := true;
	else 
		res := false;
	end if; 

end returnedHome;
