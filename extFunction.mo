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
