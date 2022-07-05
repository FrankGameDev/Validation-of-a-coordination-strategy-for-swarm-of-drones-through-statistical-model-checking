/*
QUESTO BLOCCO MODELLA L'ALGORITMO DI FLOCKING "
*/
block flockingModule 
	
	K const;

	parameter Real T = 0.5 "Timer controllo droni vicini";

	InputReal x[const.N];
	InputReal y[const.N];
	InputReal z[const.N];

	InputReal Vx[const.N];
	InputReal Vy[const.N];
	InputReal Vz[const.N];

	InputBool neighbours[const.N,const.N];

	InputInt droneState[const.N];
	InputBool droneDead[const.N];

	InputReal battery[const.N];

	OutputReal alignX[const.N];
	OutputReal alignY[const.N];
	OutputReal alignZ[const.N];

	OutputReal cohesionX[const.N];
	OutputReal cohesionY[const.N];
	OutputReal cohesionZ[const.N];
	
	OutputReal separateX[const.N];
	OutputReal separateY[const.N];
	OutputReal separateZ[const.N];
	

algorithm

when initial() then

alignX := zeros(const.N);
alignY := zeros(const.N);
alignZ := zeros(const.N);

cohesionX := zeros(const.N);
cohesionY := zeros(const.N);
cohesionZ := zeros(const.N);

separateX := zeros(const.N);
separateY := zeros(const.N);
separateZ := zeros(const.N);

end when;

when sample(0,T) then

	(alignX,alignY,alignZ) := align(const, x,y,z,Vx,Vy,Vz, droneState, droneDead, neighbours, battery);
	(cohesionX,cohesionY,cohesionZ) := cohesion(const, x,y,z,Vx,Vy,Vz, droneState, droneDead, neighbours, battery);
	(separateX,separateY,separateZ) := separate(const, x,y,z,Vx,Vy,Vz, droneState, droneDead, neighbours, battery);	

	//print("Align (" + String(alignX[1]) + ", " + String(alignY[1]) + ", " + String(alignZ[1]) + ")\n");		
	//print("Cohesion (" + String(cohesionX[1]) + ", " + String(cohesionY[1]) + ", " + String(cohesionZ[1]) + ")\n");		
	//print("Separate (" + String(separateX[1]) + ", " + String(separateY[1]) + ", " + String(separateZ[1]) + ")\n\n");		
	
end when;

end flockingModule;


function separate "Calcola la velocità di sterzata per separare ogni drone dai suoi vicini"

	input K const;

	//posizione di ogni drone
	InputReal x[const.N];
	InputReal y[const.N];
	InputReal z[const.N];

	//velocità di ogni drone
	InputReal Vx[const.N];
	InputReal Vy[const.N];
	InputReal Vz[const.N];
	
	InputInt droneState[const.N];
	InputBool droneDead[const.N];

	//Informazioni sui droni vicini
	InputBool neighbours[const.N,const.N];
	InputReal battery[const.N];	

	OutputReal outAx[const.N];
	OutputReal outAy[const.N];
	OutputReal outAz[const.N];		
	
	protected	
	//Velocità si allineamento di ogni drone con i suoi vicini. i 3 elementi indicano la velocità su x,y,z
	 Real sterring[const.N,3];

	//Numero droni vicini
	 Integer total;
	
	 Real distance;
	 Real diff[3];
	 Real separationDist = 50.0;
		

algorithm

sterring := zeros(const.N,3);
	for i in 1:const.N loop
		if(droneState[i] <> 2 and battery[i] > 0 and (not droneDead[i])) then 
			total := 0;
			//print("separate i = " + String(i)+ "\n");
			for j in 1:const.N loop
				if(neighbours[i,j] and (not droneDead[j])) then
					distance := euclideanDistance(x[i],y[i],z[i],x[j],y[j],z[j]);
					if(distance < separationDist and distance > 0) then
						diff[1] := x[i] - x[j];
						diff[2] := y[i] - y[j];
						diff[3] := z[i] - z[j];
						diff := diff / distance;
						sterring[i,1] := sterring[i,1] + diff[1];
						sterring[i,2] := sterring[i,2] + diff[2];
						sterring[i,3] := sterring[i,3] + diff[3]; 
						total := total + 1;	
						//print("cohesion i = " + String(i) + " j = " +String(j)+ " total = "+ String(total) +"\n");		
					end if;
				end if;
			end for;

			if (total > 0) then
				sterring[i,1] := sterring[i,1]/total;
				sterring[i,2] := sterring[i,2]/total;
				sterring[i,3] := sterring[i,3]/total;
				(sterring[i,1],sterring[i,2],sterring[i,3]) := norm(sterring[i,1],sterring[i,2], sterring[i,3]);
				sterring[i,1] := sterring[i,1] * const.maxSpeed;
				sterring[i,2] := sterring[i,2] * const.maxSpeed;
				sterring[i,3] := sterring[i,3] * const.maxSpeed;
				//print("moltiplicato sterring per max speed");
				sterring[i,1] := sterring[i,1] - Vx[i];
				sterring[i,2] := sterring[i,2] - Vy[i];
				sterring[i,3] := sterring[i,3] - Vz[i];
				(sterring[i,1],sterring[i,2],sterring[i,3]) := velocityCap(sterring[i,1], sterring[i,2], sterring[i,3],const.maxSpeed);
				//Limita lo sterzo alla forza massima
			end if;
		end if;
	end for;

for i in 1:const.N loop
	outAx[i] := sterring[i,1];		
	outAy[i] := sterring[i,2];	
	outAz[i] := sterring[i,3];	
end for;

end separate;


function align "Calcola, per tutti i droni, la direzione da seguire per mantenere l'allineamento con i suoi vicini"
	
	input K const;

	//posizione di ogni drone
	InputReal x[const.N];
	InputReal y[const.N];
	InputReal z[const.N];

	//velocità di ogni drone
	InputReal Vx[const.N];
	InputReal Vy[const.N];
	InputReal Vz[const.N];
	
	InputInt droneState[const.N];
	//Permette di sapere se droni,ostacoli o missili sono collisi
	InputBool droneDead[const.N];

	//Informazioni sui droni vicini
	InputBool neighbours[const.N,const.N];	
	InputReal battery[const.N];	

	OutputReal outAx[const.N];
	OutputReal outAy[const.N];
	OutputReal outAz[const.N];

	//Velocità si allineamento di ogni drone con i suoi vicini. i 3 elementi indicano la velocità su x,y,z
	protected
	 Real sterring[const.N,3];

	//Velocità media calcolata per ogni vicino di un determinato drone
	 Real avg_velocity[3];

	//Numero droni vicini
	 Integer total;

	//Distanza tra 2 drone	 
	 Real tmpMagn;

	 Real distance; //Distanza desiderata da droni vicini

	 Real droneDist = 50;
	
algorithm

	sterring := zeros(const.N,3);
	for i in 1:const.N loop
		if(droneState[i] <> 2 and battery[i] > 0 and (not droneDead[i])) then 
			total := 0;
			avg_velocity := zeros(3);
			for j in 1:const.N loop
				if(neighbours[i,j] and (not droneDead[j])) then
					/*
					Calcolo la magnitudine del vettore e lo normalizzo. Se la lunghezza è minore della distanza 
					desiderata da ciascun altro drone, aggiungo il drone seguente al totale e calcolo la velocità media
					*/
					distance := euclideanDistance(x[i],y[i],z[i],x[j],y[j],z[j]);
					if(distance <= droneDist and distance >= 0) then 
						avg_velocity := avg_velocity + {Vx[j],Vy[j],Vz[j]};
						total:= total + 1;
					end if;	
				//print("align i = " + String(i) + " j = " +String(j)+ " total = "+ String(total) +"\n");	
				end if;
			end for;
			//print("align i = " + String(i) + "\n");
			if (total > 0) then
				avg_velocity := avg_velocity/total;
				(avg_velocity[1],avg_velocity[2],avg_velocity[3]) := norm(avg_velocity[1] , avg_velocity[2] , avg_velocity[3]);
				avg_velocity := avg_velocity * const.maxSpeed;

				sterring[i,1] := avg_velocity[1] - Vx[i];
				sterring[i,2] := avg_velocity[2] - Vy[i];
				sterring[i,3] := avg_velocity[3] - Vz[i];
				(sterring[i,1],sterring[i,2],sterring[i,3]) := velocityCap(sterring[i,1], sterring[i,2], sterring[i,3],const.maxSpeed);
				//limitare lo sterzo in base alla massima forza applicabile
			end if;
		end if;
	end for;
	
	for i in 1:const.N loop
		outAx[i] := sterring[i,1];		
		outAy[i] := sterring[i,2];	
		outAz[i] := sterring[i,3];	
	end for;

end align;




function cohesion "Permette di calcolare la direzione che ogni singolo drone deve mantenere per rimanere vicino al centro della formazione"
	
	input K const;

	//posizione di ogni drone
	InputReal x[const.N];
	InputReal y[const.N];
	InputReal z[const.N];

	//velocità di ogni drone
	InputReal Vx[const.N];
	InputReal Vy[const.N];
	InputReal Vz[const.N];
	
	InputInt droneState[const.N];
	InputBool droneDead[const.N];

	//Informazioni sui droni vicini
	InputBool neighbours[const.N,const.N];		
	InputReal battery[const.N];	

	OutputReal outAx[const.N];
	OutputReal outAy[const.N];
	OutputReal outAz[const.N];
	

	protected
		Real center[3];

		Real sterring[const.N,3];	
		
		Integer total;

		Real vecToCom[3];

		Real tmpVect[3];
		
		Real distance;

		Real droneDist = 50;
algorithm

sterring := zeros(const.N,3);

for i in 1:const.N loop
	if(droneState[i] <> 2 and battery[i] > 0 and (not droneDead[i])) then 
		center := zeros(3);
		total := 0;
		for j in 1:const.N loop
			if(neighbours[i,j] and (not droneDead[j])) then	
				distance := euclideanDistance(x[i],y[i],z[i],x[j],y[j],z[j]); 
				if(distance < droneDist and distance > 0) then 	
					center := center + {x[j], y[j], z[j]};		
					total := total + 1;	
				end if;
			end if;
		end for;
		//print("cohesion i = " + String(i)+ "\n");	
		if (total > 0) then
			center := center / total;
			vecToCom := center - {x[i],y[i],z[i]};
			(vecToCom[1],vecToCom[2],vecToCom[3]) := norm(vecToCom[1],vecToCom[2],vecToCom[3]);
			vecToCom := vecToCom * const.maxSpeed;
			//print("ho calcolato avg \n");
			sterring[i,1] := vecToCom[1] - Vx[i];
			sterring[i,2] := vecToCom[2] - Vy[i];
			sterring[i,3] := vecToCom[3] - Vz[i];
			//print("ho calcolato sterring\n");
			(sterring[i,1],sterring[i,2],sterring[i,3]) := velocityCap(sterring[i,1], sterring[i,2], sterring[i,3],const.maxSpeed);
			//limita la sterzata in base alla forza massima 	
		end if;	
	end if;
end for;

for i in 1:const.N loop
	outAx[i] := sterring[i,1];		
	outAy[i] := sterring[i,2];	
	outAz[i] := sterring[i,3];	
end for;

end cohesion;	



