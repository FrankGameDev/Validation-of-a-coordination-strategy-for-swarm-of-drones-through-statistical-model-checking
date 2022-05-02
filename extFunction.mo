function velocityCap 
	
	InputReal Vx;
	InputReal Vy;
	InputReal Vz;
	InputReal velCap;

	OutputReal VxCap;
	OutputReal VyCap;
	OutputReal VzCap;

algorithm
	
	VxCap := Vx;
	VyCap := Vy;
	VzCap := Vz;

	if (abs(Vx) > velCap) then 
		VxCap := if(Vx >= 0) then velCap else -velCap;
	end if;
	if (abs(Vy) > velCap) then 
		VyCap := if(Vy > 0) then velCap else -velCap;
	end if;
	if (abs(Vz) > velCap) then 
		VzCap := if(Vz > 0) then velCap else -velCap;
	end if;
end velocityCap;

function magnitude "Restituisce la magnitudine di un vettore 3D"

	InputReal Vx;
	InputReal Vy;
	InputReal Vz;

	OutputReal res;

algorithm
		
	res := sqrt(Vx^2 + Vy^2 + Vz^2);

end magnitude;

function magnitude2D "Restituisce la magnitudine di un vettore 2D"

	InputReal Vx;
	InputReal Vy;

	OutputReal res;

algorithm
		
	res := sqrt(Vx^2 + Vy^2);

end magnitude2D;

function norm "Normalizza un vettore"

	InputReal Vx;
	InputReal Vy;
	InputReal Vz;


	OutputReal xNorm;
	OutputReal yNorm;
	OutputReal zNorm;
	
	protected
	 Real magn;

algorithm
	
	magn := magnitude(Vx,Vy,Vz);
	if(magn > 0) then
	xNorm := Vx/magn;
	yNorm := Vy/magn;
	zNorm := Vz/magn; 
	end if;

end norm;

function skyScan "Controlla i dintorni dei droni attraverso il sistema di rilevamento ad infrarossi e il sistema video"

	InputReal x;
	InputReal y;
	InputReal z;

	InputReal x2[K.N];
	InputReal y2[K.N];
	InputReal z2[K.N];	
	
	InputReal destX;
	InputReal destY; 
	InputReal destZ; 

	InputReal intrX[K.nIntr];
	InputReal intrY[K.nIntr];
	InputReal intrZ[K.nIntr];

	InputReal missX[K.nRocket];
	InputReal missY[K.nRocket];
	InputReal missZ[K.nRocket];

	InputReal statX[K.nStatObs];
	InputReal statY[K.nStatObs];
	InputReal statZ[K.nStatObs];

	OutputBool neighbours[K.N];
	OutputBool nearIntr[K.nIntr];
	OutputBool nearMissile[K.nRocket];
	OutputBool nearStatObs[K.nStatObs];

	protected 
		Boolean neigh1[K.N],neigh2[K.N];
		Boolean nIntr1[K.nIntr],nIntr2[K.nIntr];
		Boolean nMiss1[K.nRocket],nMiss2[K.nRocket];
		Boolean nStat1[K.nStatObs],nStat2[K.nStatObs];

algorithm
	(neigh1, nIntr1, nMiss1, nStat1) := findNearObject(x, y, z, x2,y2,z2, intrX, intrY, intrZ,
																		missX, missY, missZ, statX,statY,statZ);
	(neigh2, nIntr2, nMiss2, nStat2) := seeNearObject(x, y, z, x2,y2,z2, destX, destY, destZ,intrX, intrY, intrZ,
																	missX, missY, missZ, statX,statY,statZ); 
	neighbours := neigh1 or neigh2;
	nearIntr := nIntr1 or nIntr2;
	nearMissile := nMiss1 or nMiss2;
	nearStatObs := nStat1 or nStat2;
end skyScan;

function findNearObject "Restituisce una lista contenente tutti gli oggetti vicini utilizzando gli infrarossi"
	
	InputReal x;
	InputReal y;
	InputReal z;

	InputReal x2[K.N];
	InputReal y2[K.N];
	InputReal z2[K.N];

	InputReal intrX[K.nIntr];
	InputReal intrY[K.nIntr];
	InputReal intrZ[K.nIntr];

	InputReal missX[K.nRocket];
	InputReal missY[K.nRocket];
	InputReal missZ[K.nRocket];

	InputReal statX[K.nStatObs];
	InputReal statY[K.nStatObs];
	InputReal statZ[K.nStatObs];

	OutputBool neighbours[K.N];
	OutputBool nearIntr[K.nIntr];
	OutputBool nearMissile[K.nRocket];
	OutputBool nearStatObs[K.nStatObs];

	protected
		Real euclDist;

algorithm
	for i in 1:K.N loop
		euclDist := euclideanDistance(x,y,z,x2[i],y2[i],z2[i]);
		if(euclDist <= K.IDD and euclDist > 0) then
			neighbours[i] := true;
		else neighbours[i] := false;
		end if;	
	end for; 
	
	for i in 1:K.nIntr loop
		euclDist := euclideanDistance(x,y,z,intrX[i], intrY[i], intrZ[i]);
		if(euclDist <= K.IDD and euclDist > 0) then
			nearIntr[i] := true;
		else nearIntr[i] := false;
		end if;	
	end for; 

	for i in 1:K.nRocket loop
		euclDist := euclideanDistance(x,y,z,missX[i], missY[i], missZ[i]);
		if(euclDist <= K.IDD and euclDist > 0) then
			nearMissile[i] := true;
		else nearMissile[i] := false;
		end if;	
	end for; 

	for i in 1:K.nStatObs loop
		euclDist := euclideanDistance(x,y,z,statX[i], statY[i], statZ[i]);
		if(euclDist <= K.IDD and euclDist > 0) then
			nearStatObs[i] := true;
		else nearStatObs[i] := false;
		end if;	
	end for; 
end findNearObject;


function seeNearObject "Restituisce una lista contenente tutti gli oggetti rilevati dal sistema video del drone"
	
	InputReal x;
	InputReal y;
	InputReal z;

	InputReal x2[K.N];
	InputReal y2[K.N];
	InputReal z2[K.N];

	InputReal destX;
	InputReal destY; 
	InputReal destZ; 

	InputReal intrX[K.nIntr];
	InputReal intrY[K.nIntr];
	InputReal intrZ[K.nIntr];

	InputReal missX[K.nRocket];
	InputReal missY[K.nRocket];
	InputReal missZ[K.nRocket];

	InputReal statX[K.nStatObs];
	InputReal statY[K.nStatObs];
	InputReal statZ[K.nStatObs];

	OutputBool outneighbours[K.N];
	OutputBool outnearIntr[K.nIntr];
	OutputBool outnearMissile[K.nRocket];
	OutputBool outnearStatObs[K.nStatObs];

	protected
		Real viewField[3];

algorithm
	
	//Imposto i limiti del campo visivo. Data la direzione del drone, ogni asse avrà il suo limite.
	// viewField := findViewField(x,y,z, destX,destY,destZ);
	viewField := zeros(3);

	for j in 1:K.N loop
		outneighbours[j] := false;
		if((x2[j] >= x and x2[j] <= viewField[1]) or (x2[j] <= x and x2[j] >= viewField[1])) then
			if((y2[j] >= y and y2[j] <= viewField[2]) or (y2[j] <= y and y2[j] >= viewField[2])) then
				if((z2[j] >= z and z2[j] <= viewField[3]) or (z2[j] <= z and z2[j] >= viewField[3])) then
					outneighbours[j] := true;
				end if;
			end if;
		end if;
	end for;

	for j in 1:K.nIntr loop
		outnearIntr[j] := false;
		if((intrX[j] >= x and intrX[j] <= viewField[1]) or (intrX[j] <= x and intrX[j] >= viewField[1])) then
			if((intrY[j] >= y and intrY[j] <= viewField[2]) or (intrY[j] <= y and intrY[j] >= viewField[2])) then
				if((intrZ[j] >= z and intrZ[j] <= viewField[3]) or (intrZ[j] <= z and intrZ[j] >= viewField[3])) then
					outnearIntr[j] := true;
				end if;
			end if;
		end if;
	end for; 

	for j in 1:K.nRocket loop
		outnearMissile[j] := false;
		if((missX[j] >= x and missX[j] <= viewField[1]) or (missX[j] <= x and missX[j] >= viewField[1])) then
			if((missY[j] >= y and missY[j] <= viewField[2]) or (missY[j] <= y and missY[j] >= viewField[2])) then
				if((missZ[j] >= z and missZ[j] <= viewField[3]) or (missZ[j] <= z and missZ[j] >= viewField[3])) then
					outnearMissile[j] := true;
				end if;
			end if;
		end if;
	end for; 

	for j in 1:K.nStatObs loop
		outnearStatObs[j] := false;
		if((statX[j] >= x and statX[j] <= viewField[1]) or (statX[j] <= x and statX[j] >= viewField[1])) then
			if((statY[j] >= y and statY[j] <= viewField[2]) or (statY[j] <= y and statY[j] >= viewField[2])) then
				if((statZ[j] >= z and statZ[j] <= viewField[3]) or (statZ[j] <= z and statZ[j] >= viewField[3])) then
					outnearStatObs[j] := true;
				end if;
			end if;
		end if;
	end for; 

end seeNearObject;

function findViewField "Calcola la direzione di un drone normalizzando le componenti del vettore al valore 1 o -1"

	//Punto 1
	InputReal x1;
	InputReal y1;
	InputReal z1;
	
	//Punto 2
	InputReal x2;
	InputReal y2;
	InputReal z2;

	OutputReal view[3];

	protected 
		Real vect[3];
		Integer direction[3];

algorithm
	view := {x2 - x1, y2 - y1, z2 - z1};

	//Set x
	direction[1] := if(vect[1] >= 0) then 1 else -1;

	//Set y
	direction[2] := if(vect[2] >= 0) then 1 else -1;
	
	//Set z
	direction[3] := if(vect[3] >= 0) then 1 else -1; 
 
	view := {direction[1]*(x1 + K.horizontalODD), direction[2]*(y1 + K.horizontalODD), direction[3]*(z1 + K.verticalODD)};


end findViewField;

function euclideanDistance
	
	//Drone 1
	InputReal x1;
	InputReal y1;
	InputReal z1;
	
	//Drone 2
	InputReal x2;
	InputReal y2;
	InputReal z2;

	OutputReal dist; 

algorithm
	
	dist := sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2);	
	

end euclideanDistance;

function euclideanDistance2D
	//Drone 1
	InputReal x1;
	InputReal y1;
	
	//Drone 2
	InputReal x2;
	InputReal y2;

	OutputReal dist; 

algorithm
	
	dist := sqrt((x2-x1)^2 + (y2-y1)^2);	

end euclideanDistance2D;


function vectorAngle2D "calcola l'angolo tra 2 vettori 2D"

//INPUT

//Drone
InputReal x;
InputReal y;
//intruso
InputReal x2;
InputReal y2;

//OUTPUT

//Angolo 
OutputReal gamma; 


protected
    Real dotProd; //Prodotto tra 2 vettori
    Real magnProd; //Prodotto tra la magnitudine di 2 vettori

algorithm

    //Prima parte: calcolo theta
    dotProd := (x*x2) + (y*y2); 
    magnProd := magnitude2D(x,y) * magnitude2D(x2,y2);
	if(magnProd == 0) then 
		gamma := acos(0) * (180/K.pi);
	else
    	gamma := acos(dotProd/magnProd) * (180/K.pi);
	end if;

end vectorAngle2D;

function vectorAngle3D "calcola l'angolo tra 2 vettori 3D"

//INPUT

//Drone
InputReal x;
InputReal y;
InputReal z;
//intruso
InputReal x2;
InputReal y2;
InputReal z2;

//OUTPUT

//Angolo XYZ
OutputReal gamma; 

protected
    Real dotProd; //Prodotto tra 2 vettori
    Real magnProd; //Prodotto tra la magnitudine di 2 vettori

algorithm

    //Prima parte: calcolo gamma
    dotProd := (x*x2) + (y*y2) + (z*z2); 
    magnProd := magnitude(x,y,z) * magnitude(x2,y2,z2);
    gamma := acos(dotProd/magnProd) * (180/K.pi);

end vectorAngle3D;