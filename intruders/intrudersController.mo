block IntrController

K const;

parameter Real T = 0.1;   //seconds

parameter Real p = -1;
parameter Real kz1 = -(p^2);   
parameter Real kz2 = 2*p;   
parameter Real ky1 = -(p^2);   
parameter Real ky2 = 2*p;   
parameter Real kx1 = -(p^2);   
parameter Real kx2 = 2*p;   
parameter Real maxSpeed = 7.5;

InputReal setx[const.nIntr];
InputReal sety[const.nIntr];
InputReal setz[const.nIntr];


InputReal x[const.nIntr];
InputReal y[const.nIntr];
InputReal z[const.nIntr];
InputReal Vx[const.nIntr];
InputReal Vy[const.nIntr];
InputReal Vz[const.nIntr];


//Forza
OutputReal Trustx[const.nIntr];
OutputReal Trusty[const.nIntr];
OutputReal Trustz[const.nIntr];


//Calcolo il trust e la nuova posizione
algorithm

for i in 1:const.nIntr loop
	Trustx[i] := const.m*(kx1*(x[i] - setx[i]) + kx2*Vx[i]);
	Trusty[i] := const.m*(ky1*(y[i] - sety[i]) + ky2*Vy[i]);
	Trustz[i] := const.m*(const.g + kz1*(z[i] - setz[i]) + kz2*Vz[i]) - const.intrudersMass;

	(Trustx[i],Trusty[i],Trustz[i]) := velocityCap(Trustx[i],Trusty[i],Trustz[i],const.maxSpeed);	
end for;


end IntrController;