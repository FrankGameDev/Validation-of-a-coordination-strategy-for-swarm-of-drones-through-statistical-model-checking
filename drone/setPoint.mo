block SetPoint
	
	parameter Real T = 180"tempo di aggiornamento del punto di arrivo";

	Real rand[3];

	InputReal battery[K.N];
	//Posizioni di partenza dei droni
	InputReal startX[K.N], startY[K.N], startZ[K.N];

	OutputReal setx[K.N];
	OutputReal sety[K.N];
	OutputReal setz[K.N];

	
algorithm
	//Setto punto di arrivo

/* 
//posizione di arrivo totalmente random
when sample(0,T) then
	rand := myrandom();
	for i in 1:K.N loop
		if(battery[i] < (K.capacity*15)/100) then
			setx[i] := startX[i];
			sety[i] := startY[i];
			setz[i] := startZ[i];
		else
			setx[i] := myrandom() * K.flyZone[1] + K.dDistance*i;
			sety[i] := myrandom() * K.flyZone[2] + K.dDistance*i;	
			setz[i] := myrandom() * K.flyZone[3] + K.dDistance*i;
		end if;	
	end for;
end when;
 */


//arrivo in un punto prestabilito
when sample(0,T) then
	rand := {myrandom(),myrandom(),myrandom()};
	for i in 1:K.N loop
		if(battery[i] < (K.N*15)/100) then
			setx[i] := startX[i];
			sety[i] := startY[i];
			setz[i] := startZ[i];
		else
			setx[i] := rand[1] * K.flyZone[1];
			sety[i] := rand[2] * K.flyZone[2];	
			setz[i] := rand[3] * K.flyZone[3];
			if(setx[i] < K.minDestDistance) then
				setx[i] := setx[i] + K.minDestDistance;
			end if;
			if(sety[i] < K.minDestDistance) then
				sety[i] := sety[i] + K.minDestDistance;
			end if;
			if(setz[i] < K.minDestDistance) then
				setz[i] := setz[i] + K.minDestDistance;
			end if;
		end if;
	end for;
end when;


end SetPoint;
