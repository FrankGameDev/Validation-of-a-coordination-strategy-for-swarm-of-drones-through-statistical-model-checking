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

//InputReal droneWeight;

InputReal x[K.N];
InputReal y[K.N];
InputReal z[K.N];
InputReal Vx[K.N];
InputReal Vy[K.N];
InputReal Vz[K.N];

//Forza
OutputReal Trustx[K.N];
OutputReal Trusty[K.N];
OutputReal Trustz[K.N];


equation

for i in 1:K.N loop
    Trustx[i] = K.m*(kx1*(x[i] - setx[i]) + kx2*Vx[i]);
    Trusty[i] = K.m*(ky1*(y[i] - sety[i]) + ky2*Vy[i]);
    Trustz[i] = K.m*(K.g + kz1*(z[i] - setz[i]) + kz2*Vz[i]);
end for;


end Controller;

