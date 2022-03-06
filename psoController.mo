block PSO "Modulo di controllo dell'algoritmo di pathfinding"
	
//tempo di aggiornamento del pso
parameter Real T = 5;

//tendenza dei droni di rimanere alla velocità del precedente timestamp
parameter Real w = 0.6;

//tendenza dei droni di esplorare l'area circostante
parameter Real c1 = 2;

//tendenza dei droni di convergere nel punto di arrivo 
parameter Real c2 = 2;


InputReal destX[K.N];
InputReal destY[K.N];
InputReal destZ[K.N];


InputReal x[K.N];
InputReal y[K.N];
InputReal z[K.N];
InputReal Vx[K.N];
InputReal Vy[K.N];
InputReal Vz[K.N];

//global fitness e global position sono calcolate tramite il modulo di comunicazione

//Global fitnessValue. La gBest fit è unica, ma viene usata sotto forma di vettore per simulare la memorizzazione e la comunicazione della variabile per tutti i droni
InputReal InglobFitness[K.N];

//Migliore posizione globale.La gBest pos è unica, ma viene usata sotto forma di vettore per simulare la memorizzazione e la comunicazione della variabile per tutti i droni
InputReal IngBestPos[K.N,3];


//CAMPI DI OUTPUT

//Migliore posizione individuale
Real pBestPos[K.N,3];

//Migliore posizione globale.La gBest pos è unica, ma viene usata sotto forma di vettore per simulare la memorizzazione e la comunicazione della variabile per tutti i droni
OutputReal gBestPos[K.N,3];

//migliore fitness value personale
Real pBestFit[K.N];

//Global fitnessValue. La gBest fit è unica, ma viene usata sotto forma di vettore per simulare la memorizzazione e la comunicazione della variabile per tutti i droni
OutputReal globFitness[K.N];

//Variabile temporanea per il calcolo e confronto della fitness value
OutputReal tmpFit[K.N];

//valori random per calcolare velocità
Real r1;
Real r2;



OutputReal velocityX[K.N];
OutputReal velocityY[K.N];
OutputReal velocityZ[K.N];


algorithm

when initial() then
	globFitness := fill(1000000.0,K.N);
	pBestFit := fill(1000000.0, K.N);
	pBestPos := zeros(K.N,3);
	gBestPos := zeros(K.N,3);
	
	tmpFit := allFitness(x,y,z,destX,destY,destZ);	

	for i in 1:K.N loop
		
		//Confronto e setup Pbest

		if(pBestFit[i] > tmpFit[i]) then
			pBestFit[i] := tmpFit[i];
			pBestPos[i] := {x[i],y[i],z[i]};
		end if;
	
		//Aggiornamento gBests da inviare al modulo di comunicazione
		if(globFitness[i] > tmpFit[i]) then
			globFitness[i] := tmpFit[i];
			gBestPos[i] := {x[i],y[i],z[i]};
		end if;
	
	end for;
	velocityX := zeros(K.N);
	velocityY := zeros(K.N);
	velocityZ := zeros(K.N);


end when;


when sample(0,T) then

tmpFit := allFitness(x,y,z,destX,destY,destZ);	
for i in 1:K.N loop

	//Confronto e setup Pbest. Posso calcolarlo qui poichè richiede solamente i dati del singolo drone.
	if(pBestFit[i] > tmpFit[i]) then
		pBestFit[i] := tmpFit[i];
		pBestPos[i] := {x[i],y[i],z[i]};
	end if;

	//Aggiornamento gBests da inviare al modulo di comunicazione
	if(globFitness[i] > tmpFit[i]) then
		globFitness[i] := tmpFit[i];
		gBestPos[i] := {x[i],y[i],z[i]};
	end if;

	r1 := myrandom();
	r2 := myrandom();
	
	velocityX[i] := ((w*Vx[i]) + (c1*r1* (pBestPos[i,1] - x[i])) + (c2*r2* (IngBestPos[i,1] - x[i])));
	velocityY[i] := ((w*Vy[i]) + (c1*r1* (pBestPos[i,2] - y[i])) + (c2*r2* (IngBestPos[i,2] - y[i])));
	velocityZ[i] := ((w*Vz[i]) + (c1*r1* (pBestPos[i,3] - z[i])) + (c2*r2* (IngBestPos[i,3] - z[i])));
	//velocity cap
	(velocityX[i],velocityY[i],velocityZ[i]) := velocityCap(velocityX[i],velocityY[i],velocityZ[i]);
	
end for;
	

end when;

end PSO;

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

//Possibile migliorarlo aggiungendo una penalità se il percorso è ostruito da ostacoli
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
		fitValue[i] := fitness(x[i],y[i],z[i],destX[i],destY[i],destZ[i]);
	end for;

end allFitness;




