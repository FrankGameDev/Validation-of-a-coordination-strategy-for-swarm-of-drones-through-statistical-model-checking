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

	//Accellerazione del drone calcolata dal controller
	InputReal accX[K.N],accY[K.N],accZ[K.N];


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
		Vx[i] = accX[i];	
		Vy[i] = accY[i];	
		Vz[i] = accZ[i];	
	end for;

	for i in 1:K.N loop
		x[i] = x[i] + accX[i];
		y[i] = y[i] + accY[i];
		z[i] = z[i] + accZ[i];
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
	//Controllo zona di volo
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
	print("Velocity 1: (" + String(Vx[1]) + ", " + String(Vy[1]) + ", " + String(Vz[1]) + ")\n");
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

function vel

	InputReal x,y,z;

	//Forza su x
	InputReal Fx;
	//Forza su y
	InputReal Fy;
	//Forza su z
	InputReal Fz;

	InputReal alignX,alignY,alignZ;
	InputReal separateX,separateY,separateZ;
	InputReal cohesionX,cohesionY,cohesionZ;
	InputReal headingX,headingY,headingZ;

	OutputReal outx,outy,outz;

algorithm
 outx := x + Fx + (alignX + cohesionX + separateX + headingX); 
 outy := y + Fy + (alignY + cohesionY + separateY + headingY); 
 outz := z + Fz + (alignZ + cohesionZ + separateZ + headingZ); 
 
end vel;