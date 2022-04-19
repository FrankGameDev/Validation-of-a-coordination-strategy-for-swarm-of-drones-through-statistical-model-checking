block MonitorCollision"Controlla se i droni collidono con oggetti nell'area di volo"
	
	parameter Real T = 1; //Refresh controllo collisione

	//input Vector3D drones[K.N];

	InputReal x[K.N];
	InputReal y[K.N];
	InputReal z[K.N];

	InputReal intrX[K.nIntr];
	InputReal intrY[K.nIntr];
	InputReal intrZ[K.nIntr];

	Real distEucl,t;

	OutputBool outCollision;


initial algorithm
	distEucl := 0.0;
	t := 0;
	outCollision := false;

algorithm

when sample(0,T) then		
	//Dato un drone, controlla se collide con gli altri k-1 droni
	for i in 1:K.N loop //drone da controllare
		for j in 1:K.N loop //droni nelle vicinanze 		
			if(i <> j) then 			
				distEucl := euclideanDistance(x[i],y[i],z[i],x[j],y[j],z[j]);		
				//se 2 droni si trovano nella stessa posizione oppure troppo vicini, collision diventa true
				if(not outCollision) then
					outCollision := (distEucl < 1.5);
				end if; 
				if(outCollision and t < 1) then
					t := t+1;
					print("Collisione tra droni");
				end if;
			end if;
		end for;
		
		for j in 1:K.nIntr loop//Controlla le collisioni con gli intrusi
			distEucl := euclideanDistance(x[i],y[i],z[i],intrX[j], intrY[j], intrZ[j]);	
			if(not outCollision) then	
				outCollision := (distEucl < 1.5);
			end if; 
			if(outCollision and t < 1) then
				t := t+1;
				print("Collisione tra drone e ostacolo");
			end if;
		end for;
	end for;

end when;

end MonitorCollision;





				