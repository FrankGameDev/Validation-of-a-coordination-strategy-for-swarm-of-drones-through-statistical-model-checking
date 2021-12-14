
block Drone

	Parameter Real T = 1;
	
//PARAMETRI DRONE 

	//Peso del drone
	parameter Real weight = 0.895;

	//Velocità massima in salita (m/s)
	parameter Real maxUpSpeed = 6;

	//Velocità massima in discesa (m/s)
	parameter Real maxDownSpeed = 6;

	//Velocità massima di volo senza vento (m/s)
	parameter Real maxUpSpeed = 15;

	//Tempo di volo massimo (minuti)
	parameter int maxFlightTime = 46;

	//Tempo di volo massimo con vento (minuti)
	parameter int maxWindFlightTime = 40;

	//Distanza massima percorribile(Km)
	parameter int maxFlightDistance = 30;
	
	//Denota se il drone è il leader della formazione a triangolo
	bool isMaster;
	
	//Istanza del drone leader della formazione a triangolo
	leadingDrone masterDrone;

//PARAMETRI SISTEMA DI VISIONE(ODD = orizontal detection distance, IDD  = infrared detection distance}
 	
	constant parameter Real horizontalODD[2] = {0.7,40};

	constant parameter Real verticalODD[2] = {0.6, 30};

	constant parameter Real IDD[2] = {0.1, 8};

//PARAMETRI BATTERIA

	//capacità massima(mAh)	
	parameter Real capacity = 5000;
	
	//Energia massima(Wh)
	parameter Real energy = 77;

	//Tensione massima(V)
	parameter Real voltage = 15.4;

//Parametri volo

	//Posizione sull'asse x
	output Real x;

	//Posizione sull'asse y
	output Real y;
	
	//Posizione sull'asse z
	output Real z;
	
	//Forza su x
	Real Fx;
	
	//Forza su y
	Real Fy;

	//Forza su z
	Real Fz;
	
	//Velocità su x
	Real Vx;

	//Velocità su y
	Real Vy;

	//Velocità su z
	Real Vz;
initial equation

	x=0;
	y=0;
	z=0;

	Fx=1;
	Fy=1;
	Fz=1;

equation
	/*
	* f = m * a --> a =f/m
	* f= m der(v)
	* v = der(x)
	*/
	

	der(x) = Vx;
	der(Vx) = Fx/weight; 


	der(y) = Vy;
	der(Vy) = Fy/weight;

	der(z) = Vz;
	der(Vz) = Fz/weight;

end equation;

algorithm
	
	when sample(0, T) then
		Fx := (1-2*(myrandom())*pre(Fx);
		Fy := (1-2*(myrandom())*pre(Fy);
		Fz := (1-2*(myrandom())*pre(Fz);
end algorithm;

end Drone;
