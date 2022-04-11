block IntrController

parameter Real T = 0.1;   //seconds

parameter Real p = -1;
parameter Real kz1 = -(p^2);   
parameter Real kz2 = 2*p;   
parameter Real ky1 = -(p^2);   
parameter Real ky2 = 2*p;   
parameter Real kx1 = -(p^2);   
parameter Real kx2 = 2*p;   
parameter Real maxSpeed = 7.5;

InputReal setx[K.nIntr];
InputReal sety[K.nIntr];
InputReal setz[K.nIntr];


InputReal x[K.nIntr];
InputReal y[K.nIntr];
InputReal z[K.nIntr];
InputReal Vx[K.nIntr];
InputReal Vy[K.nIntr];
InputReal Vz[K.nIntr];


//Forza
OutputReal Trustx[K.nIntr];
OutputReal Trusty[K.nIntr];
OutputReal Trustz[K.nIntr];


//Calcolo il trust e la nuova posizione
algorithm

for i in 1:K.nIntr loop
	Trustx[i] := K.m*(kx1*(x[i] - setx[i]) + kx2*Vx[i]);
	Trusty[i] := K.m*(ky1*(y[i] - sety[i]) + ky2*Vy[i]);
    	Trustz[i] := K.m*(K.g + kz1*(z[i] - setz[i]) + kz2*Vz[i]);
	//(Trustx[i], Trusty[i], Trustz[i]) := velocityCap(Trustx[i], Trusty[i], Trustz[i], maxSpeed);
end for;


end IntrController;
