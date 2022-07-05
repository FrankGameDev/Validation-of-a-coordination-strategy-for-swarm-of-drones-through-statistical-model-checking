block RockController

K const;
parameter Real T = 0.1;   //seconds

parameter Real p = -1;
parameter Real kz1 = -(p^2);   
parameter Real kz2 = 2*p;   
parameter Real ky1 = -(p^2);   
parameter Real ky2 = 2*p;   
parameter Real kx1 = -(p^2);   
parameter Real kx2 = 2*p;   
parameter Real maxSpeed = 7;

InputReal setx[const.nRocket];
InputReal sety[const.nRocket];
InputReal setz[const.nRocket];


InputReal x[const.nRocket];
InputReal y[const.nRocket];
InputReal z[const.nRocket];
InputReal Vx[const.nRocket];
InputReal Vy[const.nRocket];
InputReal Vz[const.nRocket];

//Determina se il missile ha raggiunto la destinazione
InputBool targetReached[const.nRocket];

//Forza
OutputReal Trustx[const.nRocket];
OutputReal Trusty[const.nRocket];
OutputReal Trustz[const.nRocket];


//Calcolo il trust e la nuova posizione
algorithm

for i in 1:const.nRocket loop
	if(targetReached[i]) then
		Trustx[i] := 0;
		Trusty[i] := 0;
		Trustz[i] := if(z[i] > 5) then -const.rocketMass*const.g else 0;
	else
		Trustx[i] := const.m*(kx1*(x[i] - setx[i]) + kx2*Vx[i]);
		Trusty[i] := const.m*(ky1*(y[i] - sety[i]) + ky2*Vy[i]);
		Trustz[i] := const.m*(const.g + kz1*(z[i] - setz[i]) + kz2*Vz[i]) - const.rocketMass*const.g;
		(Trustx[i],Trusty[i],Trustz[i]) := velocityCap(Trustx[i],Trusty[i],Trustz[i],maxSpeed);	
	end if;

end for;


end RockController;


