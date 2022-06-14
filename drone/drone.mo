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

	//Posizione ostacoli
	InputReal statX[K.nStatObs];
	InputReal statY[K.nStatObs];
	InputReal statZ[K.nStatObs];

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

	//Vettore contente informazioni sugli ostacoli fissi
	OutputBool nearStatObs[K.N, K.nStatObs];

	//capacità della batteria di ogni drone
	OutputReal actualCapacity[K.N]; 	

	//Start position of drones
	OutputReal startPos[K.N,3];

	Real tmpBatt;

initial equation
	
	for i in 1:K.N loop
		x[i] = i*K.dDistance;
		y[i] = 0;
		z[i] = 0;
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
		der(Vx[i]) = Trustx[i];	
		der(Vy[i]) = Trusty[i];
		der(Vz[i]) = Trustz[i];	

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
		nearStatObs	:= fill(false, K.N, K.nStatObs);
	end for;
end when;

when sample(0,T) then
	//Controllo zona di volo
	for i in 1:K.N loop
		if(actualCapacity[i] > 0 and (not droneDead[i])) then
			tmpBatt := actualCapacity[i];
			actualCapacity[i] := batteryMonitor(tmpBatt,1);
			//Se i sensori del drone non funzionano allora non trova gli oggetti vicini
			if(droneState[i] <> 2) then
				(neighbours[i], nearIntr[i], nearMissile[i], nearStatObs[i]) := findNearObject(x[i], y[i], z[i],x,y,z, intrX, intrY, intrZ, missX, missY, missZ, statX,statY,statZ);				
				/* (neighbours[i], nearIntr[i], nearMissile[i], nearStatObs[i]) := skyScan(x[i], y[i], z[i],
									x,y,z, destX[i], destY[i], destZ[i], intrX, intrY, intrZ, missX, missY, missZ, statX,statY,statZ); */
				tmpBatt := actualCapacity[i];
				actualCapacity[i] := batteryMonitor(tmpBatt,2);
			else
				neighbours[i] := fill(false, K.N);	
				nearIntr[i] := fill(false, K.nIntr);
				nearMissile[i] := fill(false,K.nRocket);
				nearStatObs[i] := fill(false,K.nStatObs);
			end if;	
		end if;
	end for;
	//(neighbours, nearIntr, nearMissile) := seeNearObject(x, y, z, destX, destY,destZ, intrX, intrY, intrZ, missX, missY, missZ, neighbours, nearIntr, nearMissile);
	// print("Velocity 1: (" + String(Vx[1]) + ", " + String(Vy[1]) + ", " + String(Vz[1]) + ")\n");
end when;
	
	
end Drone;


function batteryMonitor

	InputReal battery;
	
	//Scarica della batteria
	InputReal dischargeRate;
	
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
