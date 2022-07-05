block SetPoint
	K const;

	parameter Real T = 180"tempo di aggiornamento del punto di arrivo";

	parameter Real rand[3];

	InputReal battery[const.N];
	//Posizioni di partenza dei droni
	InputReal startX[const.N], startY[const.N], startZ[const.N];

	OutputReal setx[const.N];
	OutputReal sety[const.N];
	OutputReal setz[const.N];

initial equation
rand = fill(100,3);
	
algorithm
	//Setto punto di arrivo

/* 
//posizione di arrivo totalmente random
when sample(0,T) then
	rand := myrandom();
	for i in 1:const.N loop
		if(battery[i] < (const.capacity*15)/100) then
			setx[i] := startX[i];
			sety[i] := startY[i];
			setz[i] := startZ[i];
		else
			setx[i] := myrandom() * const.flyZone[1] + const.dDistance*i;
			sety[i] := myrandom() * const.flyZone[2] + const.dDistance*i;	
			setz[i] := myrandom() * const.flyZone[3] + const.dDistance*i;
		end if;	
	end for;
end when;
 */


//arrivo in un punto prestabilito
when sample(0,T) then
	// rand := {myrandom(),myrandom(),myrandom()};
	for i in 1:const.N loop
		if(battery[i] < (const.N*15)/100) then
			setx[i] := startX[i];
			sety[i] := startY[i];
			setz[i] := startZ[i];
		else
			setx[i] := rand[1];
			sety[i] := rand[2];	
			setz[i] := rand[3];
			if(setx[i] < const.minDestDistance) then
				setx[i] := setx[i] + const.minDestDistance;
			end if;
			if(sety[i] < const.minDestDistance) then
				sety[i] := sety[i] + const.minDestDistance;
			end if;
			if(setz[i] < const.minDestDistance) then
				setz[i] := setz[i] + const.minDestDistance;
			end if;
		end if;
	end for;
end when;


end SetPoint;
