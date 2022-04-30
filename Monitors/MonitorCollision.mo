block MonitorCollision"Controlla se i droni collidono con oggetti nell'area di volo"
	
	parameter Real T = 1; //Refresh controllo collisione

	//input Vector3D drones[K.N];

	InputReal x[K.N];
	InputReal y[K.N];
	InputReal z[K.N];

	InputReal intrX[K.nIntr];
	InputReal intrY[K.nIntr];
	InputReal intrZ[K.nIntr];

	//Posizione missili
	InputReal missX[K.nRocket];
	InputReal missY[K.nRocket];
	InputReal missZ[K.nRocket];

	//Posizione ostacoli
	InputReal statX[K.nStatObs];
	InputReal statY[K.nStatObs];
	InputReal statZ[K.nStatObs];

	InputBool droneDead[K.N];
    InputBool intrDead[K.nIntr];
    InputBool missDead[K.nRocket];

	Real t;

	OutputBool outCollision;

initial algorithm
	t := 0;
	outCollision := false;

algorithm
when sample(0,T) then		
	//Dato un drone, controlla se collide con gli altri k-1 droni
	for i in 1:K.N loop //drone da controllare
		if(not droneDead[i]) then
			for j in 1:K.N loop //droni nelle vicinanze 		
				if(i <> j and (not droneDead[j])) then 			
					//se 2 droni si trovano nella stessa posizione, collision diventa true
					if(not outCollision) then
						outCollision := checkPosition(x[i],y[i],z[i],x[j],y[j],z[j]);
					end if; 
					if(outCollision and t < 1) then
						t := t+1;
					end if;
				end if;
			end for;
			
			for j in 1:K.nIntr loop//Controlla le collisioni con gli intrusi
				if((not intrDead[j]) and (not outCollision)) then	
					outCollision := checkPosition(x[i],y[i],z[i],intrX[j], intrY[j], intrZ[j]);
				end if; 
				if(outCollision and t < 1) then
					t := t+1;
				end if;
			end for;

			for j in 1:K.nRocket loop//Controlla le collisioni con gli intrusi
				if((not missDead[j]) and (not outCollision)) then
					outCollision := checkPosition(x[i],y[i],z[i],missX[j], missY[j], missZ[j]);
				end if; 
				if(outCollision and t < 1) then
					t := t+1;
				end if;
			end for;

			for j in 1:K.nStatObs loop//Controlla le collisioni con gli intrusi
				if(not outCollision) then
					outCollision := checkPosition(x[i],y[i],z[i],statX[j], statY[j], statZ[j]);
				end if; 
				if(outCollision and t < 1) then
					t := t+1;
				end if;
			end for;
		end if;
	end for;
end when;

end MonitorCollision;


function checkPosition

	InputReal x,y,z;
	InputReal x2,y2,z2;

	OutputBool col;

algorithm
	col := false;
	if(euclideanDistance(x,y,z,x2,y2,z2) < 2) then
		col := true;
	end if;
	
end checkPosition;



				
