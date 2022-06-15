block Rocket "modella dei missili che cercano di colpire i droni"
	
	K const;
//Parametri volo
	
	//Forza su x
	InputReal Trustx[const.nRocket];
	
	//Forza su y
	InputReal Trusty[const.nRocket];

	//Forza su z
	InputReal Trustz[const.nRocket];

	//Posizione sull'asse x
	OutputReal x[const.nRocket];

	//Posizione sull'asse y
	OutputReal y[const.nRocket];
	
	//Posizione sull'asse z
	OutputReal z[const.nRocket];
	
	//Velocità su x
	OutputReal Vx[const.nRocket];

	//Velocità su y
	OutputReal Vy[const.nRocket];

	//Velocità su z
	OutputReal Vz[const.nRocket];

	//Posizione di partenza dei missili
	// OutputReal startPos[const.nRocket,3];
	
initial equation
	
	//inizializzo posizione intrusi
	for i in 1:const.nRocket loop
		x[i] = myrandom() * const.flyZone[1];
		y[i] = myrandom() * const.flyZone[2];
		z[i] = 0; 
	end for;
	

	for i in 1:const.nRocket loop
		Vx[i] = 0;
		Vy[i] = 0;
		Vz[i] = 0;
	end for;	



//Aggiorno velocità e posizione dell'inturso
equation

	for i in 1:const.nRocket loop
		der(Vx[i]) = Trustx[i];	
		der(Vy[i]) = Trusty[i];
		der(Vz[i]) = Trustz[i];	

		der(x[i]) = Vx[i];
		der(y[i]) = Vy[i];
		der(z[i]) = Vz[i];
	end for;


end Rocket;


