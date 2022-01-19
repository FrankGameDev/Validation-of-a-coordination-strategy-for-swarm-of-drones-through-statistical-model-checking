/*
QUESTO BLOCCO MODELLA L'ALGORITMO DI COLLISION AVOIDANCE BASATO SU UN ALGORITMO DI FLOCKING. Si attiva solo se il monitor di collision segnala una collisione imminente"
*/
block CollisionAvoidance 
	
	parameter Real T = 0.5 "Timer controllo droni vicini";

	InputReal x[K.N];
	InputReal y[K.N];
	InputReal z[K.N];

	InputReal Vx[K.N];
	InputReal Vy[K.N];
	InputReal Vz[K.N];

	InputBool collision "Output del monitor che controlla le collisioni";

	OutputReal newVx[K.N];
	OutputReal newVy[K.N];
	OutputReal newVz[K.N];


	Real alignX[K.N];
	Real alignY[K.N];
	Real alignZ[K.N];

	Real cohesionX[K.N];
	Real cohesionY[K.N];
	Real cohesionZ[K.N];
	
	Real separateX[K.N];
	Real separateY[K.N];
	Real separateZ[K.N];
	


equation

(alignX,alignY,alignZ) = align(x,y,z,Vx,Vy,Vz);
(cohesionX,cohesionY,cohesionZ) = cohesion(x,y,z,Vx,Vy,Vz);
(separateX,separateY,separateZ) = separate(x,y,z,Vx,Vy,Vz);

for i in 1:K.N loop	
	newVx[i] = Vx[i] + alignX[i] + cohesionX[i] + separateX[i];
	newVy[i] = Vy[i] + alignY[i] + cohesionY[i] + separateY[i];
	newVz[i] = Vz[i] + alignZ[i] + cohesionZ[i] + separateZ[i];
end for;
	


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
					(diff[0],diff[1],diff[2]) := norm(diff[0],diff[1],diff[2]);
					diff := diff / distance;
					sterring[i] := sterring[i] + diff; 
					total := total + 1;		
				end if;
			end if;
		end for;

		if (total > 0) then
			sterring[i] := div(sterring[i],total);
			
		end if;
		
		if (magnitude(sterring[i,0],sterring[i,1], sterring[i,2]) > 0) then
			(sterring[i,0],sterring[i,1],sterring[i,2]) := norm(sterring[i,0],sterring[i,1], sterring[i,2]);
			sterring[i] := sterring[i] * K.maxSpeed;
			sterring[i] := sterring[i] - {Vx[i],Vy[i],Vz[i]};
			//Limita lo sterzo alla forza massima
		end if;
	end for;

for i in 1:K.N loop
	outAx[i] := sterring[i,0];		
	outAy[i] := sterring[i,1];	
	outAz[i] := sterring[i,2];	
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
			avg_velocity := div(avg_velocity,total);
			
			(avg_velocity[0],avg_velocity[1],avg_velocity[2]) := norm(avg_velocity[0] , avg_velocity[1] , avg_velocity[2]);
		
			avg_velocity := avg_velocity * K.maxSpeed;

			sterring[i] := avg_velocity - {Vx[i],Vy[i],Vz[i]};
			
			//limitare lo sterzo in base alla massima forza applicabile
		end if;
	end for;
	
for i in 1:K.N loop
	outAx[i] := sterring[i,0];		
	outAy[i] := sterring[i,1];	
	outAz[i] := sterring[i,2];	
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
		(vecToCom[0],vecToCom[1],vecToCom[2]) := norm(vecToCom[0],vecToCom[1],vecToCom[2]);
		vecToCom := vecToCom * K.maxSpeed;
		sterring[i] := vecToCom - {Vx[i],Vy[i],Vz[i]};
		//limita la sterzata in base alla forza massima 	
	end if;	
end for;

for i in 1:K.N loop
	outAx[i] := sterring[i,0];		
	outAy[i] := sterring[i,1];	
	outAz[i] := sterring[i,2];	
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
	xNorm := div(Vx,magn);
	yNorm := div(Vy,magn);
	zNorm := div(Vz,magn); 
	end if;

end norm;
