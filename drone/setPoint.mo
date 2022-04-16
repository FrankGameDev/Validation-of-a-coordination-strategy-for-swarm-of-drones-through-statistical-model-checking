block SetPoint
	
	parameter Real T = 10 "tempo di aggiornamento del punto di arrivo";

	Real rand;

	InputReal battery[K.N];
	InputReal startX[K.N], startY[K.N], startZ[K.N];

	OutputReal setx[K.N];
	OutputReal sety[K.N];
	OutputReal setz[K.N];

	
algorithm
	//Setto punto di arrivo
	
/*	
	for i in 1:K.N loop
		rand := myrandom();
		if(battery[i] < (K.N*15)/100) then
			setx[i] := startX[i];
			sety[i] := startY[i];
			setz[i] := startZ[i];
		else
			setx[i] := 0;
			sety[i] := 2*i + 5*i*sin((2*3.14/2)*time);		
			setz[i] := 3*i + 5*i*cos((2*3.14/2)*time);
		end if;
	end for;
*/

/*
//posizione di arrivo totalmente random
when sample(0,T) then
	rand := myrandom();
	for i in 1:K.N loop
		if(battery[i] < (K.N*15)/100) then
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


//arrivo in fila
when sample(0,T) then
	rand := myrandom();
	for i in 1:K.N loop
		if(battery[i] < (K.N*15)/100) then
			setx[i] := startX[i];
			sety[i] := startY[i];
			setz[i] := startZ[i];
		else
			setx[i] := rand * K.flyZone[1] + K.dDistance*i;
			sety[i] := rand * K.flyZone[2] + K.dDistance*i;	
			setz[i] := rand * K.flyZone[3] + K.dDistance*i;
		end if;
	end for;
	//print(String(setx[1]) + "\t" + String(sety[1]) + "\t" +String(setz[1]) + "\n");
end when;


end SetPoint;
