block Intruders "modella gli oggetti in movimento che ostacolano i droni"
	
	
//PARAMETRI intrusi

	//Velocità massima di volo senza vento (m/s)
	parameter Real maxSpeed = 7.5;

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
		der(Vx[i]) = Trustx[i];	
		der(Vy[i]) = Trusty[i];
		der(Vz[i]) = Trustz[i];	

		der(x[i]) = Vx[i];
		der(y[i]) = Vy[i];
		der(z[i]) = Vz[i];
	end for;
	
end Intruders;