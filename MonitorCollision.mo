block MonitorCollision
	
	parameter Real T = 0.5; //Refresh controllo collisione

	//input Vector3D drones[K.N];

	InputReal x[K.N];
	InputReal y[K.N];
	InputReal z[K.N];

	Real distEucl;

	OutputBool outCollision;


initial algorithm
	distEucl := 0.0;
	outCollision := false;

algorithm

when sample(0,T) then		
	//Dato un drone, controlla se collide con gli altri k-1 droni
	for i in 1:K.N loop //drone da controllare
		for j in 1:K.N loop //droni nelle vicinanze 		
			if(i <> j) then 			
				distEucl := euclideanDistance(x[i],y[i],z[i],x[j],y[j],z[j]);		
				//distEucl := euclideanDistance(drones[i].x,drones[i].y,drones[i].z,drones[j].x,drones[j].y,drones[j].z);		
				//Se la distanza tra 2 droni è minore della distanza massima dell'IDD, collision diventa true
				outCollision := (distEucl < K.dDistance); 
			end if;
		end for;
	end for;

end when;

end MonitorCollision;





				
