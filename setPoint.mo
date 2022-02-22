block SetPoint
	
	parameter Real T = 15 "tempo di aggiornamento del punto di arrivo";

	Real height = 50.0;
	Real lenght = 50.0;	
	Real profondity = 50.0;	
		
	Real rand;

	OutputReal setx[K.N];
	OutputReal sety[K.N];
	OutputReal setz[K.N];

algorithm
	//Setto punto di arrivo
/*	
	for i in 1:K.N loop
		setx[i] = i;
		sety[i] = i;	
		setz[i] = i+1;		
	end for;
	
	
	for i in 1:K.N loop
		setx[i] = 0;
		sety[i] = 2*i + 5*i*sin((2*3.14/2)*time);		
		setz[i] = 3*i + 5*i*cos((2*3.14/2)*time);
	end for;
*/

when sample(0,T) then
	rand := myrandom();
	for i in 1:K.N loop
		setx[i] := rand * lenght + K.dDistance + i;
		sety[i] := rand * profondity + K.dDistance + i;	
		setz[i] := rand * height + K.dDistance + i;
	end for;
	//print(String(setx[1]) + "\t" + String(sety[1]) + "\t" +String(setz[1]) + "\n");
end when;
	

end SetPoint;
