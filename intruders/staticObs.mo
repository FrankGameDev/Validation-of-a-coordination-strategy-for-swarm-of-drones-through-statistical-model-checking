block StaticObs "modella gli oggetti in movimento che ostacolano i droni"

	parameter Real T = 1000000;

	//Posizione sull'asse x
	OutputReal x[K.nStatObs];

	//Posizione sull'asse y
	OutputReal y[K.nStatObs];
	
	//Posizione sull'asse z
	OutputReal z[K.nStatObs];

initial equation
	
	x = zeros(K.nStatObs);
	y = zeros(K.nStatObs);
	z = zeros(K.nStatObs);

algorithm
	when sample(0,T) then
		for i in 1:K.nStatObs loop
			x[i] := myrandom() * K.flyZone[1];
			y[i] := myrandom() * K.flyZone[1];
			z[i] := myrandom() * K.flyZone[1];
		end for;
	end when;
end StaticObs;
