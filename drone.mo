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

	//Stato del volo dei droni. Se arrivati a destinazione diventa true, altrimenti false
	InputBool travelState;

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

//Parametri sull'applicazione degli algoritmi di flocking e Pathfinding

//La somma dei vari pesi degli attributi deve essere uguale a 1.
	parameter Real cohesionWeight = 1;	
	parameter Real alignWeight = 6;
	parameter Real separateWeight = 6;
	parameter Real headingWeight = 10;
	parameter Real vWeight = 5;
	//Real inertiaWeight;

initial equation
	
	for i in 1:K.N loop
		x[i] = 0;
		y[i] = 0;
		z[i]= 5+i;
	end for;


	for i in 1:K.N loop
		Vx[i] = 0;
		Vy[i] = 0;
		Vz[i] = 0;
	end for;

	

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
		

		der(x[i]) = Vx[i];

		der(Vx[i]) = (Fx[i]/weight)*vWeight + (alignX[i]*alignWeight + cohesionX[i]*cohesionWeight + separateX[i]*separateWeight); 
		//der(Vx[i]) = (Fx[i]/weight)*vWeight + headingX[i]*headingWeight;

		der(y[i]) = Vy[i];

		der(Vy[i]) = (Fy[i]/weight)*vWeight + (alignY[i]*alignWeight + cohesionY[i]*cohesionWeight + separateY[i]*separateWeight);
		//der(Vy[i]) = (Fy[i]/weight)*vWeight + headingY[i]*headingWeight;
		
		der(z[i]) = Vz[i];
		der(Vz[i]) = (Fz[i]/weight)*vWeight + (alignZ[i]*alignWeight + cohesionZ[i]*cohesionWeight + separateZ[i]*separateWeight);
		//der(Vz[i]) = (Fz[i]/weight)*vWeight + headingZ[i]*headingWeight;
		

//		print("Vx = " + String(Vx[i]) + " Vy = "+ String(Vy[i]) + " Vz = " + String(Vz[i]) +"\n");
	end for;


	
end Drone;

