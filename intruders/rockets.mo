block Rocket "modella dei missili che cercano di colpire i droni"
	

//Parametri volo
	
	//Forza su x
	InputReal Trustx[K.nRocket];
	
	//Forza su y
	InputReal Trusty[K.nRocket];

	//Forza su z
	InputReal Trustz[K.nRocket];

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

	//Posizione di partenza dei missili
	// OutputReal startPos[K.nRocket,3];
	
initial equation
	
	//inizializzo posizione intrusi
	for i in 1:K.nRocket loop
		x[i] = myrandom() * K.flyZone[1];
		y[i] = myrandom() * K.flyZone[2];
		z[i] = 0; 
	end for;
	

	for i in 1:K.nRocket loop
		Vx[i] = 0;
		Vy[i] = 0;
		Vz[i] = 0;
	end for;	



//Aggiorno velocità e posizione dell'inturso
equation

	for i in 1:K.nRocket loop
		der(Vx[i]) = Trustx[i];	
		der(Vy[i]) = Trusty[i];
		der(Vz[i]) = Trustz[i];	

		der(x[i]) = Vx[i];
		der(y[i]) = Vy[i];
		der(z[i]) = Vz[i];
	end for;


end Rocket;


