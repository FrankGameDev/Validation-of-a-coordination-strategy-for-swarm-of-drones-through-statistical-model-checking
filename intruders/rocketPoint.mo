block RocketPointer"Determina i punti di arrivo degli intrusi"

	parameter Real T = 10 "tempo di aggiornamento del punto di arrivo";

	//Posizione droni
	InputReal droneX[K.N], droneY[K.N], droneZ[K.N];

	InputReal x[K.nRocket],y[K.nRocket],z[K.nRocket];

	OutputReal setx[K.nRocket];
	OutputReal sety[K.nRocket];
	OutputReal setz[K.nRocket];

	Integer droneFollowed[K.nRocket];
	Real dronePosition[K.nRocket,3];

initial algorithm
	setx := zeros(K.nRocket);
	sety := zeros(K.nRocket);
	setz := zeros(K.nRocket);
	
	droneFollowed := fill(-1, K.nRocket);
	dronePosition := zeros(K.nRocket,3);

algorithm
	when sample(0,T) then
		for i in 1:K.nRocket loop
			(droneFollowed[i], dronePosition[i]) := findDrones(x[i], y[i], z[i], droneX, droneY, droneZ, droneFollowed[i], dronePosition[i]);
			if(droneFollowed[i] < 0) then
				setx[i] := myrandom() * K.flyZone[1];
				sety[i] := myrandom() * K.flyZone[2];	
				setz[i] := myrandom() * K.flyZone[3];
			else
				setx[i] := dronePosition[i,1];
				sety[i] := dronePosition[i,2];
				setz[i] := dronePosition[i,3]; 
			end if;
	
		end for;
	end when;

end RocketPointer;

function findDrones"Permette di trovare tutti i droni entro 250Km"
	
	//Posizione missile
	InputReal x,y,z;
	
	//Posizione droni
	InputReal dX[K.N], dY[K.N], dZ[K.N];

	InputInt foll;
	InputReal iP[3];

	//Drone da seguire
	OutputInt followed; 
	OutputReal pos[3];

	protected
		Real best;
		Real euclDist;

algorithm	
	best := 250.0; 
	followed := foll;
	pos := iP;
	for i in 1:K.N loop
		if(followed < 0) then
			euclDist := euclideanDistance(x,y,z,dX[i],dY[i],dZ[i]);
			if(euclDist <= 250.0 and euclDist <= best) then
				best := euclDist;
				followed := i;
				pos := {dX[followed], dY[followed], dZ[followed]};
			end if;	
		end if;
	end for; 
end findDrones;
