/*
QUESTO BLOCCO MODELLA L'ALGORITMO DI COLLISION AVOIDANCE BASATO SU UN ALGORITMO DI FLOCKING. Si attiva solo se il monitor di collision segnala una collisione imminente"
*/
block CollisionAvoidance 
	
	parameter Real T = 1.0 "Timer controllo droni vicini";

	InputReal x[K.N];
	InputReal y[K.N];
	InputReal z[K.N];

	InputReal Vx[K.N];
	InputReal Vy[K.N];
	InputReal Vz[K.N];

	InputBool collision "Output del monitor che controlla le collisioni";

	OutputReal alignX[K.N];
	OutputReal alignY[K.N];
	OutputReal alignZ[K.N];

	OutputReal cohesionX[K.N];
	OutputReal cohesionY[K.N];
	OutputReal cohesionZ[K.N];
	
	OutputReal separateX[K.N];
	OutputReal separateY[K.N];
	OutputReal separateZ[K.N];
	





algorithm

when initial() then

alignX := fill(0.0,K.N);
alignY := fill(0.0,K.N);
alignZ := fill(0.0,K.N);

cohesionX := fill(0.0,K.N);
cohesionY := fill(0.0,K.N);
cohesionZ := fill(0.0,K.N);

separateX := fill(0.0,K.N);
separateY := fill(0.0,K.N);
separateZ := fill(0.0,K.N);
end when;

when sample(0,T) then

	(alignX,alignY,alignZ) := align(x,y,z,Vx,Vy,Vz);
	(cohesionX,cohesionY,cohesionZ) := cohesion(x,y,z,Vx,Vy,Vz);
	(separateX,separateY,separateZ) := separate(x,y,z,Vx,Vy,Vz);

	
end when;

end CollisionAvoidance;


function separate "Calcola la velocità di sterzata per separare ogni drone dai suoi vicini"

	//posizione di ogni drone
	InputReal x[K.N];
	InputReal y[K.N];
	InputReal z[K.N];

	//velocità di ogni drone
	InputReal Vx[K.N];
	InputReal Vy[K.N];
	InputReal Vz[K.N];
	
	
	OutputReal outAx[K.N];
	OutputReal outAy[K.N];
	OutputReal outAz[K.N];		
	
	protected
	 Real distanzaDesiderata = 10.0 ; //Distanza desiderata da droni vicini
	
	//Velocità si allineamento di ogni drone con i suoi vicini. i 3 elementi indicano la velocità su x,y,z
	 Real sterring[K.N,3];

	//Numero droni vicini
	 Integer total;
	
	 Real distance;
	 Real diff[3];
		

algorithm

sterring := zeros(K.N,3);
	for i in 1:K.N loop
		total := 0;
		for j in 1:K.N loop
			if(i <> j) then
				distance := euclideanDistance(x[i],y[i],z[i],x[j],y[j],z[j]);
				if(distance < distanzaDesiderata and distance > 0) then
					diff := {x[i],y[i],z[i]} - {x[j],y[j],z[j]};
					(diff[1],diff[2],diff[3]) := norm(diff[1],diff[2],diff[3]);
					diff := diff / distance;
					sterring[i] := sterring[i] + diff; 
					total := total + 1;		
				end if;
			end if;
		end for;

		if (total > 0) then
			sterring[i] := sterring[i]/total;
			
		end if;
		
		if (magnitude(sterring[i,1],sterring[i,2], sterring[i,3]) > 0) then
			(sterring[i,1],sterring[i,2],sterring[i,3]) := norm(sterring[i,1],sterring[i,2], sterring[i,3]);
			sterring[i] := sterring[i] * K.maxSpeed;
			sterring[i] := sterring[i] - {Vx[i],Vy[i],Vz[i]};
			//Limita lo sterzo alla forza massima
		end if;
	end for;

for i in 1:K.N loop
	outAx[i] := sterring[i,1];		
	outAy[i] := sterring[i,2];	
	outAz[i] := sterring[i,3];	
end for;

end separate;


function align "Calcola, per tutti i droni, la direzione da seguire per mantenere l'allineamento con i suoi vicini"
	
	//posizione di ogni drone
	InputReal x[K.N];
	InputReal y[K.N];
	InputReal z[K.N];

	//velocità di ogni drone
	InputReal Vx[K.N];
	InputReal Vy[K.N];
	InputReal Vz[K.N];
	
	
	OutputReal outAx[K.N];
	OutputReal outAy[K.N];
	OutputReal outAz[K.N];

	//Velocità si allineamento di ogni drone con i suoi vicini. i 3 elementi indicano la velocità su x,y,z
	protected
	 Real sterring[K.N,3];

	//Velocità media calcolata per ogni vicino di un determinato drone
	 Real avg_velocity[3];

	//Numero droni vicini
	 Integer total;

	//Distanza tra 2 drone	 
	 Real tmpMagn;

	 Real distanzaDesiderata = 10.0 ; //Distanza desiderata da droni vicini


	
algorithm

sterring := zeros(K.N,3);
	for i in 1:K.N loop
		total := 0;
		avg_velocity := zeros(3);
		for j in 1:K.N loop
			if(i <> j) then
				/*
				Calcolo la magnitudine del vettore e lo normalizzo. Se la lunghezza è minore della distanza 
				desiderata da ciascun altro drone, aggiungo il drone seguente al totale e calcolo la velocità media
				*/
				if(euclideanDistance(x[i],y[i],z[i],x[j],y[j],z[j]) < distanzaDesiderata) then 
					avg_velocity := avg_velocity + {Vx[j],Vy[j],Vz[j]};
					total:= total + 1;
				end if;				
			end if;
		end for;

		if (total > 0) then
			avg_velocity := avg_velocity/total;
			
			(avg_velocity[1],avg_velocity[2],avg_velocity[3]) := norm(avg_velocity[1] , avg_velocity[2] , avg_velocity[3]);
		
			avg_velocity := avg_velocity * K.maxSpeed;

			sterring[i] := avg_velocity - {Vx[i],Vy[i],Vz[i]};
			
			//limitare lo sterzo in base alla massima forza applicabile
		end if;
	end for;
	
for i in 1:K.N loop
	outAx[i] := sterring[i,1];		
	outAy[i] := sterring[i,2];	
	outAz[i] := sterring[i,3];	
end for;

end align;




function cohesion "Permette di calcolare la direzione che ogni singolo drone deve mantenere per rimanere vicino al centro della formazione"
	
	//posizione di ogni drone
	InputReal x[K.N];
	InputReal y[K.N];
	InputReal z[K.N];

	//velocità di ogni drone
	InputReal Vx[K.N];
	InputReal Vy[K.N];
	InputReal Vz[K.N];
	
	
	OutputReal outAx[K.N];
	OutputReal outAy[K.N];
	OutputReal outAz[K.N];
	

	protected
		Real center[3];

		Real sterring[K.N,3];	
		
		Integer total;

		Real vecToCom[3];

		Real tmpVect[3];

algorithm

sterring := zeros(K.N,3);

for i in 1:K.N loop
	center := zeros(3);
	total := 0;
	for j in 1:K.N loop
		if(i <> j) then	
			if(euclideanDistance(x[i],y[i],z[i],x[j],y[j],z[j]) < 10) then 	
				center := center + {x[j], y[j], z[j]};		
				total := total + 1;	
			end if;
		end if;
	end for;
	
	if (total > 0) then
		center := center / total;
		vecToCom := center - {x[i],y[i],z[i]};
		(vecToCom[1],vecToCom[2],vecToCom[3]) := norm(vecToCom[1],vecToCom[2],vecToCom[3]);
		vecToCom := vecToCom * K.maxSpeed;
		sterring[i] := vecToCom - {Vx[i],Vy[i],Vz[i]};
		//limita la sterzata in base alla forza massima 	
	end if;	
end for;

for i in 1:K.N loop
	outAx[i] := sterring[i,1];		
	outAy[i] := sterring[i,2];	
	outAz[i] := sterring[i,3];	
end for;

end cohesion;	



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
