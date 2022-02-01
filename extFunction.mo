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

	if (Vx > maxSpeed) then 
		if(Vx > 0) then VxCap := K.maxSpeed;
		else VxCap := -K.maxSpeed;
		end if;	
	end if;
	if (Vy > maxSpeed) then 
		if(Vy > 0) then VyCap := K.maxSpeed;
		else VyCap := -K.maxSpeed;
		end if;	
	end if;
	if (Vz > maxSpeed) then 
		if(Vz > 0) then VzCap := K.maxSpeed;
		else VzCap := -K.maxSpeed;	
		end if;	
	end if;
end velocityCap;
