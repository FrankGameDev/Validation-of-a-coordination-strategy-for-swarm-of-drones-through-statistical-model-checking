block RocketPointer"Determina i punti di arrivo degli intrusi"
	K const;
	parameter Real T = 1 "tempo di aggiornamento del punto di arrivo";

	//Posizione droni
	InputReal droneX[const.N], droneY[const.N], droneZ[const.N];

	InputReal x[const.nRocket],y[const.nRocket],z[const.nRocket];

	OutputReal setx[const.nRocket];
	OutputReal sety[const.nRocket];
	OutputReal setz[const.nRocket];

	OutputBool targetReached[const.nRocket];

	Integer droneFollowed[const.nRocket];
	Integer dIndex;
	
	//Timer di inseguimento impostato a 5 secondi
	Real timer[const.nRocket];

initial algorithm
	for i in 1:const.N loop
		setx[i] := myrandom() * const.flyZone[1];
		sety[i] := myrandom() * const.flyZone[2];	
		setz[i] := myrandom() * const.flyZone[3];
	end for;
	
	droneFollowed := fill(-1, const.nRocket);
	targetReached := fill(false, const.nRocket);
	timer := fill(5,const.nRocket);

algorithm
	when sample(0,T) then
		for i in 1:const.nRocket loop
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
	InputReal dX[const.N], dY[const.N], dZ[const.N];

	InputInt foll;

	//Drone da seguire
	OutputInt followed; 

	protected
		Real best;
		Real euclDist;
		K const;

algorithm	
	best := const.detectionDistance; 
	followed := foll;
	for i in 1:const.N loop
		if(foll < 0) then
			euclDist := euclideanDistance(x,y,z,dX[i],dY[i],dZ[i]);
			if(euclDist <= const.detectionDistance and euclDist <= best) then
				best := euclDist;
				followed := i;
			end if;	
		end if;
	end for; 
end findDrones;
