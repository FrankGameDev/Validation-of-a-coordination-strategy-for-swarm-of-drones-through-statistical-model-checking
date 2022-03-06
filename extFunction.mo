function velocityCap 
	
	InputReal Vx;
	InputReal Vy;
	InputReal Vz;

	OutputReal VxCap;
	OutputReal VyCap;
	OutputReal VzCap;

algorithm
	
	VxCap := Vx;
	VyCap := Vy;
	VzCap := Vz;

	if (Vx > K.maxSpeed) then 
		if(Vx > 0) then VxCap := K.maxSpeed;
		else VxCap := -K.maxSpeed;
		end if;	
	end if;
	if (Vy > K.maxSpeed) then 
		if(Vy > 0) then VyCap := K.maxSpeed;
		else VyCap := -K.maxSpeed;
		end if;	
	end if;
	if (Vz > K.maxSpeed) then 
		if(Vz > 0) then VzCap := K.maxSpeed;
		else VzCap := -K.maxSpeed;	
		end if;	
	end if;
end velocityCap;


function magnitude "Restituisce la magnitudine di un vettore"

	InputReal Vx;
	InputReal Vy;
	InputReal Vz;

	OutputReal res;

algorithm
		
	res := sqrt(Vx^2 + Vy^2 + Vz^2);

end magnitude;

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

function findNearDrones "Restituisce una lista contenente tutti i droni vicini data una distanza prestabilita"
	
	InputReal x;
	InputReal y;
	InputReal z;

	InputReal x2[K.N];
	InputReal y2[K.N];
	InputReal z2[K.N];

	OutputBool neighbours[K.N];

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

end findNearDrones;

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
	
	dist := sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2);	
	

end euclideanDistance;
