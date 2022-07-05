block Drone
	
	K const;

	parameter Real T = 0.5;

//PARAMETRI BATTERIA

	//capacità massima(mAh)	
	parameter Real capacity = 5000;

//Parametri volo
	
	//Destinazione droni
	InputReal destX[const.N];
	InputReal destY[const.N];
	InputReal destZ[const.N];

	//forza di movimento del drone
	InputReal Trustx[const.N];
	InputReal Trusty[const.N];
	InputReal Trustz[const.N];	

	//Campi di fault

	//Stato del drone. 1 = funzionante; 2 = errore sensoristica; 3 = errore di manovra; 4 = errore comunicazioni;
	InputInt droneState[const.N];

	//Permette di sapere se droni,ostacoli o missili sono collisi
	InputBool droneDead[const.N];
    InputBool intrDead[const.nIntr];
    InputBool missDead[const.nRocket];
	
	//Campi posizione intrusi

	InputReal intrX[const.nIntr];
	InputReal intrY[const.nIntr];
	InputReal intrZ[const.nIntr];

	//Posizione missili
	InputReal missX[const.nRocket];
	InputReal missY[const.nRocket];
	InputReal missZ[const.nRocket];

	//Posizione ostacoli
	InputReal statX[const.nStatObs];
	InputReal statY[const.nStatObs];
	InputReal statZ[const.nStatObs];

	//Scarica della batteria dovuta all'utilizzo del modulo di comunicazione
	InputReal commDischarge[const.N];

	//Valore che permette di definire se il modulo di collision avoidance ha determinato
	//una nuova destinazione per schivare gli ostacoli
	InputBool useTMPDest[const.N];

	//Posizione sull'asse x
	OutputReal x[const.N];

	//Posizione sull'asse y
	OutputReal y[const.N];
	
	//Posizione sull'asse z
	OutputReal z[const.N];
	
	//Velocità su x
	OutputReal Vx[const.N];

	//Velocità su y
	OutputReal Vy[const.N];

	//Velocità su z
	OutputReal Vz[const.N];

	//Array contenente informazioni sui vicini al drone i-esimo
	OutputBool neighbours[const.N,const.N];

	//Vettore contenente informazioni sugli intrusi vicini
	OutputBool nearIntr[const.N, const.nIntr];

	//Vettore contenente informazioni sui missili vicini
	OutputBool nearMissile[const.N, const.nRocket];

	//Vettore contente informazioni sugli ostacoli fissi
	OutputBool nearStatObs[const.N, const.nStatObs];

	//capacità della batteria di ogni drone
	OutputReal actualCapacity[const.N]; 	

	//Start position of drones
	OutputReal startPos[const.N,3];

	Real tmpBatt;

initial equation
	
	for i in 1:const.N loop
		x[i] = i*const.dDistance;
		y[i] = 0;
		z[i] = 0;
	end for;


	for i in 1:const.N loop
		Vx[i] = 0;
		Vy[i] = 0;
		Vz[i] = 0;
	end for;	

	actualCapacity = fill(capacity,const.N);
	tmpBatt = actualCapacity[1];

equation
	/*
	* f = m * a --> a =f/m
	* f= m der(v)
	* v = der(x)
	*/
	
	
	for i in 1:const.N loop 
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
	for i in 1:const.N loop
		neighbours[i] := fill(true, const.N);
		startPos[i,1] := x[i];
		startPos[i,2] := y[i];
		startPos[i,3] := z[i];
		nearIntr := fill(false, const.N, const.nIntr);
		nearMissile	:= fill(false, const.N, const.nRocket);
		nearStatObs	:= fill(false, const.N, const.nStatObs);
	end for;
end when;

when sample(0,T) then
	//Controllo zona di volo
	for i in 1:const.N loop
		if(actualCapacity[i] > 0 and (not droneDead[i])) then
			tmpBatt := actualCapacity[i];
			actualCapacity[i] := batteryMonitor(tmpBatt,1);
			//Se i sensori del drone non funzionano allora non trova gli oggetti vicini
			if(droneState[i] <> 2) then
				(neighbours[i], nearIntr[i], nearMissile[i], nearStatObs[i]) := findNearObject(const,x[i], y[i], z[i],x,y,z, intrX, intrY, intrZ, missX, missY, missZ, statX,statY,statZ);				
				/* (neighbours[i], nearIntr[i], nearMissile[i], nearStatObs[i]) := skyScan(x[i], y[i], z[i],
									x,y,z, destX[i], destY[i], destZ[i], intrX, intrY, intrZ, missX, missY, missZ, statX,statY,statZ); */
				tmpBatt := actualCapacity[i];
				actualCapacity[i] := batteryMonitor(tmpBatt,2);
			else
				neighbours[i] := fill(false, const.N);	
				nearIntr[i] := fill(false, const.nIntr);
				nearMissile[i] := fill(false,const.nRocket);
				nearStatObs[i] := fill(false,const.nStatObs);
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
