block Intruders "modella gli oggetti in movimento che ostacolano i droni"
	
	
//PARAMETRI intrusi

	//Velocità massima di volo senza vento (m/s)
	parameter Real maxSpeed = 7.5;

	Real Fx[K.nIntr],Fy[K.nIntr],Fz[K.nIntr];

//Parametri volo
	
	//Forza su x
	InputReal Trustx[K.nIntr];
	
	//Forza su y
	InputReal Trusty[K.nIntr];

	//Forza su z
	InputReal Trustz[K.nIntr];


	//Posizione sull'asse x
	OutputReal x[K.nIntr];

	//Posizione sull'asse y
	OutputReal y[K.nIntr];
	
	//Posizione sull'asse z
	OutputReal z[K.nIntr];
	
	//Velocità su x
	OutputReal Vx[K.nIntr];

	//Velocità su y
	OutputReal Vy[K.nIntr];

	//Velocità su z
	OutputReal Vz[K.nIntr];

initial algorithm
	
	//inizializzo posizione intrusi
	for i in 1:K.nIntr loop
		x[i] := myrandom() * K.flyZone[1];
		y[i] := myrandom() * K.flyZone[2];
		z[i] := myrandom() * K.flyZone[3]; 
	end for;


	for i in 1:K.nIntr loop
		Vx[i] := 0;
		Vy[i] := 0;
		Vz[i] := 0;
	end for;	


//Aggiorno velocità e posizione dell'inturso
equation

	for i in 1:K.nIntr loop 
		Fx[i] = Trustx[i];	
		Fy[i] = Trusty[i];
		Fz[i] = Trustz[i] - K.m * K.g;	
	end for;

	for i in 1:K.nIntr loop 
		der(x[i]) = Vx[i];
		der(Vx[i]) = (Fx[i]/K.m); 

		der(y[i]) = Vy[i];
		der(Vy[i]) = (Fy[i]/K.m);
		
		der(z[i]) = Vz[i];
		der(Vz[i]) = (Fz[i]/K.m);
	end for;
end Intruders;


