block RocketPointer"Determina i punti di arrivo degli intrusi"

	parameter Real T = 1 "tempo di aggiornamento del punto di arrivo";

	//Posizione droni
	InputReal droneX[K.N], droneY[K.N], droneZ[K.N];

	InputReal x[K.nRocket],y[K.nRocket],z[K.nRocket];

	OutputReal setx[K.nRocket];
	OutputReal sety[K.nRocket];
	OutputReal setz[K.nRocket];

	OutputBool targetReached[K.nRocket];

	Integer droneFollowed[K.nRocket];
	Integer dIndex;
	
	//Timer di inseguimento impostato a 5 secondi
	Real timer[K.nRocket];

initial algorithm
	for i in 1:K.N loop
		setx[i] := myrandom() * K.flyZone[1];
		sety[i] := myrandom() * K.flyZone[2];	
		setz[i] := myrandom() * K.flyZone[3];
	end for;
	
	droneFollowed := fill(-1, K.nRocket);
	targetReached := fill(false, K.nRocket);
	timer := fill(5,K.nRocket);

algorithm
	when sample(0,T) then
		for i in 1:K.nRocket loop
			(droneFollowed[i]) := findDrones(x[i], y[i], z[i], droneX, droneY, droneZ, droneFollowed[i]);
			if(droneFollowed[i] > 0 and timer[i] > 0) then
				timer[i] := timer[i] - 1;
				dIndex := droneFollowed[i];
				setx[i] := droneX[dIndex];
				sety[i] := droneY[dIndex];
				setz[i] := droneZ[dIndex]; 
			end if;
			if(timer[i] <= 0) then
				targetReached[i] := true;
			end if;
		end for;
	end when;

end RocketPointer;

function findDrones "Permette di trovare tutti i droni entro il raggio di rilevamento dei missili e seguire quello più vicino"
	
	//Posizione missile
	InputReal x,y,z;
	
	//Posizione droni
	InputReal dX[K.N], dY[K.N], dZ[K.N];

	InputInt foll;

	//Drone da seguire
	OutputInt followed; 

	protected
		Real best;
		Real euclDist;

algorithm	
	best := K.detectionDistance; 
	followed := foll;
	for i in 1:K.N loop
		if(foll < 0) then
			euclDist := euclideanDistance(x,y,z,dX[i],dY[i],dZ[i]);
			if(euclDist <= K.detectionDistance and euclDist <= best) then
				best := euclDist;
				followed := i;
			end if;	
		end if;
	end for; 
end findDrones;
