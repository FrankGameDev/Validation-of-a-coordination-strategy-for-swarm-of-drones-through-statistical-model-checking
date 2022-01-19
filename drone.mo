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


	InputReal Trustx[K.N];
	InputReal Trusty[K.N];
	InputReal Trustz[K.N];

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

	//output Vector3D position[K.N];

	//peso drone in output
	//OutputReal outWeight;


	//Real sterring[3];

	//Real tmpSterring[3];


initial equation
	
	for i in 1:K.N loop
		x[i] = 0;
		y[i] = 0;
		z[i]= 5+i;
		//position[i](x = 0, y = 0, z = 5+i);
	end for;
	
	for i in 1:K.N loop
		Vx[i] = 0;
		Vy[i] = 0;
		Vz[i] = 0;
	end for;

	//outWeight = weight;


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
		//der(position[i].x) = Vx[i];
		der(Vx[i]) = Fx[i]/weight; 
		
		der(y[i]) = Vy[i];
		//der(position[i].y) = Vy[i];
		der(Vy[i]) = Fy[i]/weight;
	
		der(z[i]) = Vz[i];
		//der(position[i].z) = Vz[i];
		der(Vz[i]) = Fz[i]/weight;
		
	end for;


	
end Drone;

