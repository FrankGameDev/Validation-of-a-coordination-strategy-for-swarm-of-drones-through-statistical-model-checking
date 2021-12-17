block SetPoint

	OutputReal setx[K.N];
	OutputReal sety[K.N];
	OutputReal setz[K.N];

equation
	//Setto punto di arrivo
	/*
	for i in 1:K.n loop
		setx[i] = i;
		sety[i] = i;	
		setz[i] = i+1;		
	end for;
	*/
	for i in 1:K.N loop
		setx[i] = 0;
		sety[i] = 2*i + 5*i*sin((2*3.14/2)*time);		
		setz[i] = 3*i + 5*i*cos((2*3.14/2)*time);
	end for;
	
end SetPoint;