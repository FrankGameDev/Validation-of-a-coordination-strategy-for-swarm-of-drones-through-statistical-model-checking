block Drone
	
	parameter Real T = 1.0;
	
//PARAMETRI DRONE 

	//Peso del drone
	parameter Real weight = 0.895;

	//Velocità massima in salita (m/s)
	parameter Real maxUpSpeed = 6.0;

	//Velocità massima in discesa (m/s)
	parameter Real maxDownSpeed = 6.0;

	//Velocità massima di volo senza vento (m/s)
	parameter Real maxSpeed = 15.0;

	//Tempo di volo massimo (minuti)
	parameter Integer maxFlightTime = 46;

	//Tempo di volo massimo con vento (minuti)
	parameter Integer maxWindFlightTime = 40;

	//Distanza massima percorribile(Km)
	parameter Integer maxFlightDistance = 30;
	
//PARAMETRI SISTEMA DI VISIONE(ODD = orizontal detection distance, IDD  = infrared detection distance}
 	
	parameter Real horizontalODD[2] = {0.7,40};

	parameter Real verticalODD[2] = {0.6, 30};

	parameter Real IDD[2] = {0.1, 8};

//PARAMETRI BATTERIA

	//capacità massima(mAh)	
	parameter Real capacity = 5000;
	
	//Energia massima(Wh)
	parameter Real energy = 77;

	//Tensione massima(V)
	parameter Real voltage = 15.4;

//Parametri volo

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
	
	
	//Campi posizione intrusi

	InputReal intrX[K.nIntr];
	InputReal intrY[K.nIntr];
	InputReal intrZ[K.nIntr];

	//Scarica della batteria dovuta all'utilizzo del modulo di comunicazione
	InputReal commDischarge[K.N];

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

	OutputReal actualCapacity[K.N]; 	

	//Start position of drones
	OutputReal startPos[K.N,3];

	Real tmpBatt;

//Parametri sull'applicazione degli algoritmi di flocking e Pathfinding

//La somma dei vari pesi degli attributi deve essere uguale a 1.
	parameter Real cohesionWeight = 1;	
	parameter Real alignWeight = 1.5;
	parameter Real separateWeight = 1.5;
	parameter Real headingWeight = 1;
	parameter Real vWeight = 5;

initial equation
	
	for i in 1:K.N loop
		x[i] = 0;
		y[i] = 0;
		z[i]= 5+i*K.dDistance;
	end for;


	for i in 1:K.N loop
		Vx[i] = 0;
		Vy[i] = 0;
		Vz[i] = 0;
	end for;	

	actualCapacity = fill(capacity,K.N);
equation
	/*
	* f = m * a --> a =f/m
	* f= m der(v)
	* v = der(x)
	*/
	
	
	for i in 1:K.N loop 
		Fx[i] = Trustx[i];	
		Fy[i] = Trusty[i];
		Fz[i] = Trustz[i] - weight * K.g;	
	end for;


	for i in 1:K.N loop
	
		if(not droneState[i] == 3) then	
			
			der(Vx[i]) = (Fx[i]/weight)*vWeight + (alignX[i]*alignWeight + cohesionX[i]*cohesionWeight + separateX[i]*separateWeight + headingX[i]*headingWeight); 
			der(Vy[i]) = (Fy[i]/weight)*vWeight + (alignY[i]*alignWeight + cohesionY[i]*cohesionWeight + separateY[i]*separateWeight + headingY[i]*headingWeight);
			der(Vz[i]) = (Fz[i]/weight)*vWeight + (alignZ[i]*alignWeight + cohesionZ[i]*cohesionWeight + separateZ[i]*separateWeight + headingZ[i]*headingWeight);

			der(x[i]) = Vx[i];
			der(y[i]) = Vy[i];
			der(z[i]) = Vz[i];
		
		else
			if(z[i] <= 0) then 
				der(Vz[i]) = 0;
				der(Vy[i]) = 0;
				der(Vx[i]) = 0; 
			else
				der(Vy[i]) = 0;
				der(Vx[i]) = 0; 	
				der(Vz[i]) = -K.g*vWeight;
			end if;
			der(x[i]) = Vx[i];
			der(y[i]) = Vy[i];
			der(z[i]) = Vz[i];
		end if;
	end for;



algorithm
when initial() then
	for i in 1:K.N loop
		neighbours[i] := fill(true, K.N);
		startPos[i,1] := x[i];
		startPos[i,2] := y[i];
		startPos[i,3] := z[i];
		nearIntr := fill(false, K.N, K.nIntr);
	end for;
end when;

when sample(0,T) then
	//print(String(commDischarge[1])+"\n");
	for i in 1:K.N loop
		tmpBatt := actualCapacity[i];
		actualCapacity[i] := batteryMonitor(tmpBatt,1);
		//Se i sensori del drone non funzionano allora non trova gli oggetti vicini
		if(droneState[i] <> 2 and droneState[i] <> 3) then
			(neighbours[i], nearIntr[i]) := findNearObject(x[i], y[i], z[i], x,y,z, intrX, intrY, intrZ);
			tmpBatt := actualCapacity[i];
			actualCapacity[i] := batteryMonitor(tmpBatt,2);
		else
			neighbours[i] := fill(false, K.N);	
			nearIntr[i] := fill(false, K.nIntr);
		end if;	
	end for;
	
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
	
	if(battery == 0) then 
		outBattery := 0;
	else
		outBattery := battery - dischargeRate;
	end if;
end batteryMonitor;

/*
function move "Calcola la nuova posizione di un drone"

	InputReal Vx, Vy, Vz;
	InputReal vCap;

	OutputReal x,y,z;

algorithm
	if(abs((Vx)) > K.maxSpeed) then
		if((Vx) >= 0) then
			(x) := K.maxSpeed;
		else 
			(x) := -K.maxSpeed;
		end if;
	else 
		(x) := Vx;
	end if;

	if(abs((Vy)) > K.maxSpeed) then
		if((Vy) >= 0) then
			(y) := K.maxSpeed;
		else 
			(y) := -K.maxSpeed;
		end if;
	else
		(y) := Vy;
	end if;

	if(abs((Vz)) > K.maxSpeed) then
		if((Vz) >= 0) then
			(z) := K.maxSpeed;
		else 
			(z) := -K.maxSpeed;
		end if;
	else
		(z) := Vz;
	end if;
end move;


*/