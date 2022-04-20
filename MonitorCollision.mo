block MonitorCollision"Controlla se i droni collidono con oggetti nell'area di volo"
	
	parameter Real T = 1; //Refresh controllo collisione

	//input Vector3D drones[K.N];

	InputReal x[K.N];
	InputReal y[K.N];
	InputReal z[K.N];

	InputReal intrX[K.nIntr];
	InputReal intrY[K.nIntr];
	InputReal intrZ[K.nIntr];

	Real t;

	OutputBool outCollision;


initial algorithm
	t := 0;
	outCollision := false;

algorithm

when sample(0,T) then		
	//Dato un drone, controlla se collide con gli altri k-1 droni
	for i in 1:K.N loop //drone da controllare
		for j in 1:K.N loop //droni nelle vicinanze 		
			if(i <> j) then 			
				//se 2 droni si trovano nella stessa posizione, collision diventa true
				if(not outCollision) then
					outCollision := checkPosition(x[i],y[i],z[i],x[j],y[j],z[j]);
				end if; 
				if(outCollision and t < 1) then
					t := t+1;
					print("Collisione tra droni (" + String(i) + ", " + String(j) + ")\n");
				end if;
			end if;
		end for;
		
		for j in 1:K.nIntr loop//Controlla le collisioni con gli intrusi
			if(not outCollision) then	
				outCollision := checkPosition(x[i],y[i],z[i],intrX[j], intrY[j], intrZ[j]);
			end if; 
			if(outCollision and t < 1) then
				t := t+1;
				print("Collisione tra drone e ostacolo(" + String(i) + ", " + String(j) + ")");
			end if;
		end for;
	end for;

end when;

end MonitorCollision;


function checkPosition

	InputReal x,y,z;
	InputReal x2,y2,z2;

	OutputBool col;

algorithm
	col := false;
	if (x == x2 and y == y2 and z == z2) then
		col := true;
	end if;

end checkPosition;



				
