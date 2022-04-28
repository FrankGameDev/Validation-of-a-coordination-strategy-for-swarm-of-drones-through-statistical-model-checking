block Rocket "modella dei missili che cercano di colpire i droni"
	
	
//PARAMETRI intrusi

	//Velocità massima di volo senza vento (m/s)
	parameter Real maxSpeed = 7.5;

	Real Fx[K.nRocket],Fy[K.nRocket],Fz[K.nRocket];

//Parametri volo
	
	//Forza su x
	InputReal Trustx[K.nRocket];
	
	//Forza su y
	InputReal Trusty[K.nRocket];

	//Forza su z
	InputReal Trustz[K.nRocket];

	//Determina se il missile ha raggiunto la destinazione
	InputBool targetReached[K.nRocket];

	//Posizione sull'asse x
	OutputReal x[K.nRocket];

	//Posizione sull'asse y
	OutputReal y[K.nRocket];
	
	//Posizione sull'asse z
	OutputReal z[K.nRocket];
	
	//Velocità su x
	OutputReal Vx[K.nRocket];

	//Velocità su y
	OutputReal Vy[K.nRocket];

	//Velocità su z
	OutputReal Vz[K.nRocket];

initial algorithm
	
	//inizializzo posizione intrusi
	for i in 1:K.nRocket loop
		x[i] := myrandom() * K.flyZone[1];
		y[i] := myrandom() * K.flyZone[2];
		z[i] := myrandom() * K.flyZone[3]; 
	end for;


	for i in 1:K.nRocket loop
		Vx[i] := 0;
		Vy[i] := 0;
		Vz[i] := 0;
	end for;	


//Aggiorno velocità e posizione dell'inturso
equation

	for i in 1:K.nRocket loop 
		Fx[i] = Trustx[i];	
		Fy[i] = Trusty[i];
		Fz[i] = Trustz[i] - K.m * K.g;	
	end for;

	for i in 1:K.nRocket loop
		if(targetReached[i]) then
			der(Vx[i]) = 0; 
			der(Vy[i]) = 0;
			der(Vz[i]) = 0;
		else
			der(Vx[i]) = (Fx[i]/K.m); 
			der(Vy[i]) = (Fy[i]/K.m);
			der(Vz[i]) = (Fz[i]/K.m);
		end if; 

		der(x[i]) = Vx[i];
		der(y[i]) = Vy[i];
		der(z[i]) = Vz[i];

	end for;
end Rocket;


