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
		if(Vx >= 0) then VxCap := velCap;
		else VxCap := -velCap;
		end if;	
	end if;
	if (abs(Vy) > velCap) then 
		if(Vy > 0) then VyCap := velCap;
		else VyCap := -velCap;
		end if;	
	end if;
	if (abs(Vz) > velCap) then 
		if(Vz > 0) then VzCap := velCap;
		else VzCap := -velCap;	
		end if;	
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

	OutputBool neighbours[K.N];
	OutputBool nearIntr[K.nIntr];
	OutputBool nearMissile[K.nRocket];

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

	for i in 1:K.nIntr loop
		euclDist := euclideanDistance(x,y,z,intrX[i], intrY[i], intrZ[i]);
		if(euclDist <= K.IDD and euclDist > 0) then
			nearMissile[i] := true;
		else nearMissile[i] := false;
		end if;	
	end for; 

end findNearObject;

function seeNearObject "Restituisce una lista contenente tutti gli oggetti rilevati dal sistema video del drone"
	InputReal x;
	InputReal y;
	InputReal z;

	InputReal destX;
	InputReal destY; 
	InputReal destZ; 

	InputReal x2[K.N];
	InputReal y2[K.N];
	InputReal z2[K.N];

	InputReal intrX[K.nIntr];
	InputReal intrY[K.nIntr];
	InputReal intrZ[K.nIntr];

	InputReal missX[K.nRocket];
	InputReal missY[K.nRocket];
	InputReal missZ[K.nRocket];

	/*OutputBool neighbours[K.N];
	OutputBool nearIntr[K.nIntr];
	OutputBool nearMissile[K.nRocket];
*/
	protected
		Real euclDist;
		Real direction[3];
		Real viewField[3];

algorithm
	//Calcolo la direzione del drone, così da poter valutare il campo visivo della videocamera sul drone
	direction := {destX - x, destY - y, destZ - z};
	(direction[1], direction[2], direction[3]) := norm(direction[1],direction[2],direction[3]);
	//Imposto i limiti del campo visivo. Data la direzione del drone, ogni asse avrà il suo limite.
	viewField := {direction[1] * (x + K.horizontalODD),direction[2] * (y + K.horizontalODD), direction[3] * (z + K.horizontalODD)};
	print("Direzione drone:" + String(direction[1]) + ", "+ String(direction[2]) + ", "+ String(direction[3]) + ")\n" +
	"Campo visivo: (" + String(viewField[1]) + ", "+ String(viewField[2]) + ", "+ String(viewField[3]) + ")\n");


end seeNearObject;

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