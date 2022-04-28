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

//Destinazione temporanea in caso di imminente collisione
InputReal tmpSetX[K.N];
InputReal tmpSetY[K.N];
InputReal tmpSetZ[K.N];
//Permette di decidere quale punto di arrivo utilizzare nel calcolo del trust
InputBool useTMPDest[K.N];

//Valori restituiti dal  module
InputReal alignX[K.N];
InputReal alignY[K.N];
InputReal alignZ[K.N];

InputReal cohesionX[K.N];
InputReal cohesionY[K.N];
InputReal cohesionZ[K.N];

InputReal separateX[K.N];
InputReal separateY[K.N];
InputReal separateZ[K.N];

//direzione da prendere in aggiunta alla velocità, generata dal PSO

InputReal headingX[K.N];
InputReal headingY[K.N];
InputReal headingZ[K.N];


//Forza
Real Trustx[K.N];
Real Trusty[K.N];
Real Trustz[K.N];

Real destX,destY,destZ;

OutputReal accX[K.N],accY[K.N],accZ[K.N];
	
parameter Real cohesionWeight = 1;	
parameter Real alignWeight = 6;
parameter Real separateWeight =6;
parameter Real headingWeight = 10;
parameter Real vWeight = 5;

initial equation
accX = zeros(K.N);
accY = zeros(K.N);
accZ = zeros(K.N);

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
		
		(accX[i],accY[i], accZ[i]) := velocityCap(accX[i],accY[i], accZ[i], K.maxSpeed);
	end for;

equation
	for i in 1:K.N loop
		Trustx[i] = K.m*(kx1*(x[i] - destX) + kx2*accX[i]);
		Trusty[i] = K.m*(ky1*(y[i] - destY) + ky2*accY[i]);
		Trustz[i] = K.m*(K.g + kz1*(z[i] - destZ) + kz2*accZ[i]);
		
		accX[i] = (Trustx[i] / K.m)*vWeight + (alignX[i]*alignWeight + cohesionX[i]*cohesionWeight + separateX[i]*separateWeight + headingX[i]*headingWeight);
		accY[i] = (Trusty[i] / K.m)*vWeight + (alignY[i]*alignWeight + cohesionY[i]*cohesionWeight + separateY[i]*separateWeight + headingY[i]*headingWeight);
		accZ[i] = (Trustz[i] / (K.m*K.g))*vWeight + (alignZ[i]*alignWeight + cohesionZ[i]*cohesionWeight + separateZ[i]*separateWeight + headingZ[i]*headingWeight);

	
	end for;

	print("acceleration 1: (" + String(accX[1]) + ", " + String(accY[1]) + ", " + String(accZ[1]) + ")\n");
	

end Controller;



