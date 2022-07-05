block IntrudersPoint"Determina i punti di arrivo degli intrusi"
	
	K const;
	parameter Real T = 25 "tempo di aggiornamento del punto di arrivo";

	OutputReal setx[const.nIntr];
	OutputReal sety[const.nIntr];
	OutputReal setz[const.nIntr];

initial algorithm
setx := zeros(const.nIntr);
sety := zeros(const.nIntr);
setz := zeros(const.nIntr);


algorithm
	when sample(0,T) then
		for i in 1:const.nIntr loop
			setx[i] := myrandom() * const.flyZone[1];
			sety[i] := myrandom() * const.flyZone[2];	
			setz[i] := myrandom() * const.flyZone[3];
		end for;
	end when;

end IntrudersPoint;
