block Controller

parameter Real T = 0.001;   //seconds

parameter Real p = -1;
parameter Real kz1 = -(p^2);   
parameter Real kz2 = 2*p;   
parameter Real ky1 = -(p^2);   
parameter Real ky2 = 2*p;   
parameter Real kx1 = -(p^2);   
parameter Real kx2 = 2*p;   

InputReal setx[K.N];
InputReal sety[K.N];
InputReal setz[K.N];

InputReal x[K.N];
InputReal y[K.N];
InputReal z[K.N];
InputReal Vx[K.N];
InputReal Vy[K.N];
InputReal Vz[K.N];

//Destinazione temporanea in caso di imminente collisione
InputReal tmpSetX[K.N];
InputReal tmpSetY[K.N];
InputReal tmpSetZ[K.N];
//Permette di decidere quale punto di arrivo utilizzare nel calcolo del trust
InputBool useTMPDest[K.N];

//Forza
OutputReal Trustx[K.N];
OutputReal Trusty[K.N];
OutputReal Trustz[K.N];

Real destX,destY,destZ;

algorithm

	for i in 1:K.N loop
		if(useTMPDest[i]) then
			destX := tmpSetX[i];
			destY := tmpSetY[i];
			destZ := tmpSetZ[i];
		else
			destX := setx[i];
			destY := sety[i];
			destZ := setz[i];
		end if;

		Trustx[i] := K.m*(kx1*(x[i] - destX) + kx2*Vx[i]);
		Trusty[i] := K.m*(ky1*(y[i] - destY) + ky2*Vy[i]);
		Trustz[i] := K.m*(K.g + kz1*(z[i] - destZ) + kz2*Vz[i]);
	end for;

end Controller;

