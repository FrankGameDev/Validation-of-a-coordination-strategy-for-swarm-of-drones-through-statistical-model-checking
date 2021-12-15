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
	
	//Denota se il drone è il leader della formazione a triangolo
	//bool isMaster;
	
	//Istanza del drone leader della formazione a triangolo
	//leadingDrone masterDrone;

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

	//Posizione sull'asse x
	OutputReal x;

	//Posizione sull'asse y
	OutputReal y;
	
	//Posizione sull'asse z
	OutputReal z;
	
	//Forza su x
	OutputReal Fx;
	
	//Forza su y
	OutputReal Fy;

	//Forza su z
	OutputReal Fz;
	
	//Velocità su x
	OutputReal Vx;

	//Velocità su y
	OutputReal Vy;

	//Velocità su z
	OutputReal Vz;

initial equation

	x=10;
	y=10;
	z=10;

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
	when (Vx > 15.0) then 
		reinit(Vx,15.0);
	end when;	
	
	der(y) = Vy;
	der(Vy) = Fy/weight;
	when (Vy > 6.0) then 
		reinit(Vy,6.0);
	end when;	

	der(z) = Vz;
	der(Vz) = Fz/weight;
	when (Vz > 6.0) then 
		reinit(Vz,15.0);
	end when;	
	
	
algorithm
	
	when sample(0,T) then
		/*if(time < 1.6) then
			Fy := startDrone(pre(Fy));
		else*/
			Fx := (myrandom())*pre(Fx);
			Fy := (1-2*myrandom())*pre(Fy);
			Fz := (1-2*myrandom())*pre(Fz);
		//end if;	
	end when;
	

end Drone;

/*
block StartDroneController
	//Porta il drone ad alzarsi 10 metri da terra
	//LA velocità massima in salita è di 6 m/s, ma l'accellerazione deve essere un iperbole

	InputReal Fy;

	InputReal y;

	algorithm
		if(y <= 10) then
			Fy := (1-2*myrandom())*pre(Fy);
		end if;

end StartDroneController;



block FlyingController

	parameter Real T = 1;
	
	InputReal Fx;
	InputReal Fy;
	InputReal Fz;

	algorithm
		when sample(0, T) then
			Fx := (1-2*myrandom())*pre(Fx);
			Fy := (1-2*myrandom())*pre(Fy);
			Fz := (1-2*myrandom())*pre(Fz);
		end when;

end FlyingController;
*/

