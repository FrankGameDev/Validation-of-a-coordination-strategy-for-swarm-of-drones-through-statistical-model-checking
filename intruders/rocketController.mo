block RockController

parameter Real T = 0.1;   //seconds

parameter Real p = -1;
parameter Real kz1 = -(p^2);   
parameter Real kz2 = 2*p;   
parameter Real ky1 = -(p^2);   
parameter Real ky2 = 2*p;   
parameter Real kx1 = -(p^2);   
parameter Real kx2 = 2*p;   
parameter Real maxSpeed = 7.5;

InputReal setx[K.nRocket];
InputReal sety[K.nRocket];
InputReal setz[K.nRocket];


InputReal x[K.nRocket];
InputReal y[K.nRocket];
InputReal z[K.nRocket];
InputReal Vx[K.nRocket];
InputReal Vy[K.nRocket];
InputReal Vz[K.nRocket];


//Forza
OutputReal Trustx[K.nRocket];
OutputReal Trusty[K.nRocket];
OutputReal Trustz[K.nRocket];


//Calcolo il trust e la nuova posizione
algorithm

for i in 1:K.nRocket loop
	Trustx[i] := K.m*(kx1*(x[i] - setx[i]) + kx2*Vx[i]);
	Trusty[i] := K.m*(ky1*(y[i] - sety[i]) + ky2*Vy[i]);
    	Trustz[i] := K.m*(K.g + kz1*(z[i] - setz[i]) + kz2*Vz[i]);
end for;


end RockController;


