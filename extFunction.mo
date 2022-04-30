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
	
	InputReal x[K.N];
	InputReal y[K.N];
	InputReal z[K.N];

	InputReal destX[K.N];
	InputReal destY[K.N]; 
	InputReal destZ[K.N]; 

	InputReal intrX[K.nIntr];
	InputReal intrY[K.nIntr];
	InputReal intrZ[K.nIntr];

	InputReal missX[K.nRocket];
	InputReal missY[K.nRocket];
	InputReal missZ[K.nRocket];

	InputBool neighbours[K.N, K.N];
	InputBool nearIntr[K.N, K.nIntr];
	InputBool nearMissile[K.N, K.nRocket];

	OutputBool outneighbours[K.N, K.N];
	OutputBool outnearIntr[K.N,K.nIntr];
	OutputBool outnearMissile[K.N,K.nRocket];


	protected
		Real direction[3];
		Real viewField[3];

algorithm
	
	for i in 1:K.N loop
		//Controllo video
		//Calcolo la direzione del drone, così da poter valutare il campo visivo della videocamera sul drone
		direction := findDirection(x[i],y[i],z[i],destX[i],destY[i],destZ[i]);
		//Imposto i limiti del campo visivo. Data la direzione del drone, ogni asse avrà il suo limite.
		viewField := {direction[1]*(x[i] + K.horizontalODD), direction[2]*(y[i] + K.horizontalODD), direction[3]*(z[i] + K.verticalODD)};

		for j in 1:K.N loop
			if(not neighbours[i,j]) then
				outneighbours[i,j] := false;
				if((x[j] >= x[i] and x[j] <= viewField[1]) or (x[j] <= x[i] and x[j] >= viewField[1])) then
					if((y[j] >= y[i] and y[j] <= viewField[2]) or (y[j] <= y[i] and y[j] >= viewField[2])) then
						if((z[j] >= z[i] and z[j] <= viewField[3]) or (z[j] <= z[i] and z[j] >= viewField[3])) then
							outneighbours[i,j] := true;
						end if;
					end if;
				end if;
			end if;
		end for;

		for j in 1:K.nIntr loop
			if(not nearIntr[i,j]) then
				outnearIntr[i,j] := false;
				if((intrX[j] >= x[i] and intrX[j] <= viewField[1]) or (intrX[j] <= x[i] and intrX[j] >= viewField[1])) then
					if((intrY[j] >= y[i] and intrY[j] <= viewField[2]) or (intrY[j] <= y[i] and intrY[j] >= viewField[2])) then
						if((intrZ[j] >= z[i] and intrZ[j] <= viewField[3]) or (intrZ[j] <= z[i] and intrZ[j] >= viewField[3])) then
							outnearIntr[i,j] := true;
						end if;
					end if;
				end if;
			end if;
		end for; 

		for j in 1:K.nRocket loop
			if(not nearMissile[i,j]) then
				outnearMissile[i,j] := false;
				if((missX[j] >= x[i] and missX[j] <= viewField[1]) or (missX[j] <= x[i] and missX[j] >= viewField[1])) then
					if((missY[j] >= y[i] and missY[j] <= viewField[2]) or (missY[j] <= y[i] and missY[j] >= viewField[2])) then
						if((missZ[j] >= z[i] and missZ[j] <= viewField[3]) or (missZ[j] <= z[i] and missZ[j] >= viewField[3])) then
							outnearMissile[i,j] := true;
						end if;
					end if;
				end if;
			end if;	
		end for; 
	end for;

end seeNearObject;

function findDirection "Calcola la direzione di un drone normalizzando le componenti del vettore al valore 1 o -1"

	//Punto 1
	InputReal x1;
	InputReal y1;
	InputReal z1;
	
	//Punto 2
	InputReal x2;
	InputReal y2;
	InputReal z2;

	OutputReal direction[3];

	protected 
		Real vect[3];

algorithm
	vect := {x2 - x1, y2 - y1, z2 - z1};
	//Set x
	if(vect[1] >= 0) then
		direction[1] := 1;
	else
		direction[1] := -1;
	end if;
	//Set y
	if(vect[2] >= 0) then
		direction[2] := 1;
	else
		direction[2] := -1;
	end if;
	//Set z
	if(vect[3] >= 0) then
		direction[3] := 1;
	else
		direction[3] := -1;
	end if;

end findDirection;

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