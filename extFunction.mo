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

	InputReal x2[const.N];
	InputReal y2[const.N];
	InputReal z2[const.N];	
	
	InputReal destX;
	InputReal destY; 
	InputReal destZ; 

	InputReal intrX[const.nIntr];
	InputReal intrY[const.nIntr];
	InputReal intrZ[const.nIntr];

	InputReal missX[const.nRocket];
	InputReal missY[const.nRocket];
	InputReal missZ[const.nRocket];

	InputReal statX[const.nStatObs];
	InputReal statY[const.nStatObs];
	InputReal statZ[const.nStatObs];

	OutputBool neighbours[const.N];
	OutputBool nearIntr[const.nIntr];
	OutputBool nearMissile[const.nRocket];
	OutputBool nearStatObs[const.nStatObs];

	protected 
		K const;
		Boolean neigh1[const.N],neigh2[const.N];
		Boolean nIntr1[const.nIntr],nIntr2[const.nIntr];
		Boolean nMiss1[const.nRocket],nMiss2[const.nRocket];
		Boolean nStat1[const.nStatObs],nStat2[const.nStatObs];

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
	
	input K const;

	InputReal x;
	InputReal y;
	InputReal z;

	InputReal x2[const.N];
	InputReal y2[const.N];
	InputReal z2[const.N];

	InputReal intrX[const.nIntr];
	InputReal intrY[const.nIntr];
	InputReal intrZ[const.nIntr];

	InputReal missX[const.nRocket];
	InputReal missY[const.nRocket];
	InputReal missZ[const.nRocket];

	InputReal statX[const.nStatObs];
	InputReal statY[const.nStatObs];
	InputReal statZ[const.nStatObs];

	OutputBool neighbours[const.N];
	OutputBool nearIntr[const.nIntr];
	OutputBool nearMissile[const.nRocket];
	OutputBool nearStatObs[const.nStatObs];

	protected
		Real euclDist;

algorithm
	for i in 1:const.N loop
		euclDist := euclideanDistance(x,y,z,x2[i],y2[i],z2[i]);
		if(euclDist <= const.IDD and euclDist > 0) then
			neighbours[i] := true;
		else neighbours[i] := false;
		end if;	
	end for; 
	
	for i in 1:const.nIntr loop
		euclDist := euclideanDistance(x,y,z,intrX[i], intrY[i], intrZ[i]);
		if(euclDist <= const.IDD and euclDist > 0) then
			nearIntr[i] := true;
		else nearIntr[i] := false;
		end if;	
	end for; 

	for i in 1:const.nRocket loop
		euclDist := euclideanDistance(x,y,z,missX[i], missY[i], missZ[i]);
		if(euclDist <= const.IDD and euclDist > 0) then
			nearMissile[i] := true;
		else nearMissile[i] := false;
		end if;	
	end for; 

	for i in 1:const.nStatObs loop
		euclDist := euclideanDistance(x,y,z,statX[i], statY[i], statZ[i]);
		if(euclDist <= const.IDD and euclDist > 0) then
			nearStatObs[i] := true;
		else nearStatObs[i] := false;
		end if;	
	end for; 
end findNearObject;


function seeNearObject "Restituisce una lista contenente tutti gli oggetti rilevati dal sistema video del drone"
	
	InputReal x;
	InputReal y;
	InputReal z;

	InputReal x2[const.N];
	InputReal y2[const.N];
	InputReal z2[const.N];

	InputReal destX;
	InputReal destY; 
	InputReal destZ; 

	InputReal intrX[const.nIntr];
	InputReal intrY[const.nIntr];
	InputReal intrZ[const.nIntr];

	InputReal missX[const.nRocket];
	InputReal missY[const.nRocket];
	InputReal missZ[const.nRocket];

	InputReal statX[const.nStatObs];
	InputReal statY[const.nStatObs];
	InputReal statZ[const.nStatObs];

	OutputBool outneighbours[const.N];
	OutputBool outnearIntr[const.nIntr];
	OutputBool outnearMissile[const.nRocket];
	OutputBool outnearStatObs[const.nStatObs];

	protected
		K const;
		Real viewField[3];

	algorithm
	
	//Imposto i limiti del campo visivo. Data la direzione del drone, ogni asse avrà il suo limite.
	// viewField := findViewField(x,y,z, destX,destY,destZ);
	viewField := zeros(3);

	for j in 1:const.N loop
		outneighbours[j] := false;
		if((x2[j] >= x and x2[j] <= viewField[1]) or (x2[j] <= x and x2[j] >= viewField[1])) then
			if((y2[j] >= y and y2[j] <= viewField[2]) or (y2[j] <= y and y2[j] >= viewField[2])) then
				if((z2[j] >= z and z2[j] <= viewField[3]) or (z2[j] <= z and z2[j] >= viewField[3])) then
					outneighbours[j] := true;
				end if;
			end if;
		end if;
	end for;

	for j in 1:const.nIntr loop
		outnearIntr[j] := false;
		if((intrX[j] >= x and intrX[j] <= viewField[1]) or (intrX[j] <= x and intrX[j] >= viewField[1])) then
			if((intrY[j] >= y and intrY[j] <= viewField[2]) or (intrY[j] <= y and intrY[j] >= viewField[2])) then
				if((intrZ[j] >= z and intrZ[j] <= viewField[3]) or (intrZ[j] <= z and intrZ[j] >= viewField[3])) then
					outnearIntr[j] := true;
				end if;
			end if;
		end if;
	end for; 

	for j in 1:const.nRocket loop
		outnearMissile[j] := false;
		if((missX[j] >= x and missX[j] <= viewField[1]) or (missX[j] <= x and missX[j] >= viewField[1])) then
			if((missY[j] >= y and missY[j] <= viewField[2]) or (missY[j] <= y and missY[j] >= viewField[2])) then
				if((missZ[j] >= z and missZ[j] <= viewField[3]) or (missZ[j] <= z and missZ[j] >= viewField[3])) then
					outnearMissile[j] := true;
				end if;
			end if;
		end if;
	end for; 

	for j in 1:const.nStatObs loop
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
		K const;
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
 
	view := {direction[1]*(x1 + const.horizontalODD), direction[2]*(y1 + const.horizontalODD), direction[3]*(z1 + const.verticalODD)};


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
	K const;
algorithm

    //Prima parte: calcolo theta
    dotProd := (x*x2) + (y*y2); 
    magnProd := magnitude2D(x,y) * magnitude2D(x2,y2);
	if(magnProd == 0) then 
		gamma := acos(0) * (180/const.pi);
	else
    	gamma := acos(dotProd/magnProd) * (180/const.pi);
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
	K const;
algorithm

    //Prima parte: calcolo gamma
    dotProd := (x*x2) + (y*y2) + (z*z2); 
    magnProd := magnitude(x,y,z) * magnitude(x2,y2,z2);
    gamma := acos(dotProd/magnProd) * (180/const.pi);

end vectorAngle3D;