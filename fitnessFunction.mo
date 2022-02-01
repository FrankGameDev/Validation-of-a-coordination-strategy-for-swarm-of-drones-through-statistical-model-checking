//Valuta il valore di fitness del singolo drone
function fitness
//posizione droni
InputReal x;
InputReal y;
InputReal z;

//posizione destinazione
InputReal destX;
InputReal destY;
InputReal destZ;

OutputReal fitvalue;

algorithm
	fitvalue := euclideanDistance(x,y,z,destX,destY,destZ);
	
end fitness;

//Valuta, in base alla distanza con il punto di arrivo, un punteggio per migliorare l'ottimizzazione del pso
//più è basso, più è ottimizzato
function allFitness

//posizione droni
InputReal x[K.N];
InputReal y[K.N];
InputReal z[K.N];

//posizione destinazione
InputReal destX[K.N];
InputReal destY[K.N];
InputReal destZ[K.N];

OutputReal fitValue[K.N];

algorithm 
	
	for i in 1:K.N loop
		fitValue[i] := euclideanDistance(x[i],y[i],z[i],destX[i],destY[i],destZ[i]);
	end for;

end allFitness;
