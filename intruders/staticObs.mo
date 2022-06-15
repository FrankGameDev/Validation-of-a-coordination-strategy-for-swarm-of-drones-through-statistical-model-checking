block StaticObs "modella gli oggetti in movimento che ostacolano i droni"
	K const;
	parameter Real T = 1000000;

	//Posizione sull'asse x
	OutputReal x[const.nStatObs];

	//Posizione sull'asse y
	OutputReal y[const.nStatObs];
	
	//Posizione sull'asse z
	OutputReal z[const.nStatObs];

initial equation
	
	x = zeros(const.nStatObs);
	y = zeros(const.nStatObs);
	z = zeros(const.nStatObs);

algorithm
	when sample(0,T) then
		for i in 1:const.nStatObs loop
			x[i] := myrandom() * const.flyZone[1];
			y[i] := myrandom() * const.flyZone[1];
			z[i] := myrandom() * const.flyZone[1];
		end for;
	end when;
end StaticObs;
