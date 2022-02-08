block PSO
	
//tempo di aggiornamento del pso
parameter Real T = 1;

//tendenza dei droni di rimanere alla velocità del precedente timestamp
parameter Real w = 0.6;

//tendenza dei droni di esplorare l'area circostante
parameter Real c1 = 2;

//tendenza dei droni di convergere nel punto di arrivo 
parameter Real c2 = 2;


InputReal destX[K.N];
InputReal destY[K.N];
InputReal destZ[K.N];


InputReal x[K.N];
InputReal y[K.N];
InputReal z[K.N];
InputReal Vx[K.N];
InputReal Vy[K.N];
InputReal Vz[K.N];

//Migliore posizione individuale
Real bestPos[K.N,3];

//Migliore posizione globale
Real bestSwarmPos[K.N,3];

//drones fitness value
Real swarmFitness[K.N];

//migliore fitness value personale
Real best_droneFit[K.N];

//Global fitnessValue
Real globFitness;

//valori random per calcolare velocità
Real r1;
Real r2;

OutputReal velocityX[K.N];
OutputReal velocityY[K.N];
OutputReal velocityZ[K.N];


algorithm

when initial() then
	globFitness := 1000000.0;
	swarmFitness := allFitness(x,y,z,destX,destY,destZ);
	best_droneFit := swarmFitness;
	
	for i in 1:K.N loop
		if (swarmFitness[i] < globFitness) then
			globFitness := swarmFitness[i];
			for j in 1:K.N loop
				bestSwarmPos[j,1] := x[i];
				bestSwarmPos[j,2] := y[i];
				bestSwarmPos[j,3] := z[i];
			end for;
		end if;
		bestPos[i,1] := x[i];
		bestPos[i,2] := y[i];
		bestPos[i,3] := z[i];
	end for;
	velocityX := zeros(K.N);
	velocityY := zeros(K.N);
	velocityZ := zeros(K.N);

	r1 := 0.5;
	r2 := 0.4;

end when;


when sample(0.5,T) then
	for i in 1:K.N loop
		
		//r1 := myrandom();
		//r2 := myrandom();
		
		velocityX[i] := ((w*Vx[i]) + (c1*r1* (bestPos[i,1] - x[i])) + (c2*r2* (bestSwarmPos[i,1] - x[i])));
		velocityY[i] := ((w*Vy[i]) + (c1*r1* (bestPos[i,2] - y[i])) + (c2*r2* (bestSwarmPos[i,2] - y[i])));
		velocityZ[i] := ((w*Vz[i]) + (c1*r1* (bestPos[i,3] - z[i])) + (c2*r2* (bestSwarmPos[i,3] - z[i])));
		// velocity cap
		//(velocityX[i],velocityY[i],velocityZ[i]) := velocityCap(velocityX[i],velocityY[i],velocityZ[i]);
		
		swarmFitness[i] := fitness(x[i],y[i],z[i],destX[i],destY[i],destZ[i]);

		//Se la nuova posizione è la migliore(pbest) per tutti i droni, allora sostituiscila
		if (swarmFitness[i] < best_droneFit[i]) then
			best_droneFit[i] := swarmFitness[i];
			for j in 1:K.N loop
				bestPos[j,1] := x[i];
				bestPos[j,2] := y[i];
				bestPos[j,3] := z[i];
			end for;

		end if;
		//Se la nuova posizione è la migliore del gruppo(gbest), allora modifico
		if (swarmFitness[i] < globFitness) then
			globFitness := swarmFitness[i];
			for j in 1:K.N loop
				bestSwarmPos[j,1] := x[i];
				bestSwarmPos[j,2] := y[i];
				bestSwarmPos[j,3] := z[i];
			end for;
		end if;	
	end for;
end when;

end PSO;



