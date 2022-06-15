block Intruders "modella gli oggetti in movimento che ostacolano i droni"
	
	K const;
//Parametri volo
	
	//Forza su x
	InputReal Trustx[const.nIntr];
	
	//Forza su y
	InputReal Trusty[const.nIntr];

	//Forza su z
	InputReal Trustz[const.nIntr];


	//Posizione sull'asse x
	OutputReal x[const.nIntr];

	//Posizione sull'asse y
	OutputReal y[const.nIntr];
	
	//Posizione sull'asse z
	OutputReal z[const.nIntr];
	
	//Velocità su x
	OutputReal Vx[const.nIntr];

	//Velocità su y
	OutputReal Vy[const.nIntr];

	//Velocità su z
	OutputReal Vz[const.nIntr];

initial algorithm
	
	//inizializzo posizione intrusi
	for i in 1:const.nIntr loop
		x[i] := myrandom() * const.flyZone[1];
		y[i] := myrandom() * const.flyZone[2];
		z[i] := myrandom() * const.flyZone[3]; 
	end for;


	for i in 1:const.nIntr loop
		Vx[i] := 0;
		Vy[i] := 0;
		Vz[i] := 0;
	end for;	


//Aggiorno velocità e posizione dell'inturso
equation

	for i in 1:const.nIntr loop 
		der(Vx[i]) = Trustx[i];	
		der(Vy[i]) = Trusty[i];
		der(Vz[i]) = Trustz[i];	

		der(x[i]) = Vx[i];
		der(y[i]) = Vy[i];
		der(z[i]) = Vz[i];
	end for;
	
end Intruders;