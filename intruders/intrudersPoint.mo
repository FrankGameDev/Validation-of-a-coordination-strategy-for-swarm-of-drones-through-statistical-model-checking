block IntrudersPoint"Determina i punti di arrivo degli intrusi"

	parameter Real T = 10 "tempo di aggiornamento del punto di arrivo";

	OutputReal setx[K.nIntr];
	OutputReal sety[K.nIntr];
	OutputReal setz[K.nIntr];

initial algorithm
setx := zeros(K.nIntr);
sety := zeros(K.nIntr);
setz := zeros(K.nIntr);


algorithm
	when sample(0,T) then
		for i in 1:K.nIntr loop
			setx[i] := myrandom() * K.flyZone[1];
			sety[i] := myrandom() * K.flyZone[2];	
			setz[i] := myrandom() * K.flyZone[3];
		end for;
	end when;

end IntrudersPoint;
