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

InputInt droneState[K.N];

InputBool neighbours[K.N,K.N];

InputReal intrX[K.nIntr];
InputReal intrY[K.nIntr];
InputReal intrZ[K.nIntr];

InputBool nearIntr[K.N, K.nIntr];

//global fitness e global position sono calcolate tramite il modulo di comunicazione

//CAMPI DI OUTPUT

//Migliore posizione individuale
Real pBestPos[K.N,3];

//Migliore posizione globale.La gBest pos è unica, ma viene usata sotto forma di vettore per simulare la memorizzazione e la comunicazione della variabile per tutti i droni
OutputReal gBestPos[K.N,3];

//migliore fitness value personale
Real pBestFit[K.N];

//Global fitnessValue. La gBest fit è unica, ma viene usata sotto forma di vettore per simulare la memorizzazione e la comunicazione della variabile per tutti i droni.
//La fitness value è un valore che identifica il livello di ottimizzazione del sistema simulato
OutputReal globFitness[K.N];

//Variabile temporanea per il calcolo e confronto della fitness value
OutputReal tmpFit[K.N];

//valori random per calcolare velocità
Real r1;
Real r2;

//Variabili temporanee per salvataggio dati risultanti dal modulo di comunicazione
Real tmpGPos[K.N,3];
Real tmpGFit[K.N];

OutputReal velocityX[K.N];
OutputReal velocityY[K.N];
OutputReal velocityZ[K.N];

initial algorithm 
	globFitness := zeros(K.N);
	pBestFit := zeros(K.N);
	pBestPos := zeros(K.N,3);
	gBestPos := zeros(K.N,3);
	
	tmpFit := allFitness(x,y,z,intrX,intrY,intrZ, nearIntr);	

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

	tmpGFit := globFitness;
	tmpGPos := gBestPos;

	r1 := 0;
	r2 := 0;

algorithm

when sample(0,T) then
tmpFit := allFitness(x,y,z,intrX,intrY,intrZ, nearIntr);	
for i in 1:K.N loop

	//Confronto e setup Pbest. Posso calcolarlo qui poichè richiede solamente i dati del singolo drone.
	if(pBestFit[i] < tmpFit[i]) then
		pBestFit[i] := tmpFit[i];
		pBestPos[i] := {x[i],y[i],z[i]};
	end if;

	//Aggiornamento gBests da inviare al modulo di comunicazione
	if(globFitness[i] < tmpFit[i]) then
		globFitness[i] := tmpFit[i];
		gBestPos[i] := {x[i],y[i],z[i]};
	end if;
	
	
	r1 := myrandom();
	r2 := myrandom();
	
	velocityX[i] := ((w*Vx[i]) + (c1*r1* (pBestPos[i,1] - x[i])) + (c2*r2* (gBestPos[i,1] - x[i])));
	velocityY[i] := ((w*Vy[i]) + (c1*r1* (pBestPos[i,2] - y[i])) + (c2*r2* (gBestPos[i,2] - y[i])));
	velocityZ[i] := ((w*Vz[i]) + (c1*r1* (pBestPos[i,3] - z[i])) + (c2*r2* (gBestPos[i,3] - z[i])));
	//velocity cap
	(velocityX[i],velocityY[i],velocityZ[i]) := velocityCap(velocityX[i],velocityY[i],velocityZ[i], K.maxSpeed);
	
end for;
	(tmpGPos, tmpGFit) := talking(globFitness, gBestPos, neighbours, droneState);
	globFitness := tmpGFit;
	gBestPos := tmpGPos;

for i in 1:K.N loop
print(" i = " + String(i) + " --> Fit: " + String(globFitness[i])+ " x: " + String(gBestPos[i,1]) + " y: " + String(gBestPos[i,2]) + " z: " + String(gBestPos[i,3])+"\n"); 
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
InputReal intrX;
InputReal intrY;
InputReal intrZ;

OutputReal fitvalue;

algorithm
	fitvalue := euclideanDistance(x,y,z,intrX,intrY,intrZ);
	
end fitness;

//La fitness value di un drone viene calcolata basandosi sulla distanza con un ostacolo in movimento (drone nemico, missile, ecc...)
function allFitness

//posizione droni
InputReal x[K.N];
InputReal y[K.N];
InputReal z[K.N];

//posizione destinazione
InputReal intrX[K.nIntr];
InputReal intrY[K.nIntr];
InputReal intrZ[K.nIntr];
InputBool nearIntr[K.N, K.nIntr];

OutputReal fitValue[K.N];

algorithm 
	
	for i in 1:K.N loop
		for j in 1:K.nIntr loop
			if (nearIntr[i,j]) then
				fitValue[i] := fitness(x[i],y[i],z[i],intrX[j], intrY[j], intrZ[j]);
			end if;
		end for;
	end for;

end allFitness;

//SEZIONE RELATIVA ALLE FUNZIONI DI COMUNICAZIONE PER IL PSO

function talking "Comunicazione relativa alle info del pso"
	
	InputReal gBestFit[K.N];

	InputReal gBestPos[K.N,3];
	
	//Matrice contenente info sui droni vicini tra loro
	InputBool neighbours[K.N,K.N];

	InputInt droneState[K.N];
	
	OutputReal outGbestPos[K.N,3];

	OutputReal outGbestFit[K.N];


algorithm
outGbestPos := gBestPos;
outGbestFit := gBestFit;


for i in 1:K.N loop	
		for j in 1:K.N loop
			if(neighbours[i,j] and i <> j) then
				if(acknowledgment(droneState[i],droneState[j])) then
					if(psoComm(gBestFit[i], gBestFit[j])) then //Il drone j ha risposto con valori migliori
						outGbestFit[j] := outGbestFit[i];
						outGbestPos[j,1] := outGbestPos[i,1];
						outGbestPos[j,2] := outGbestPos[i,2];
						outGbestPos[j,3] := outGbestPos[i,3];
					else
						outGbestFit[i] := outGbestFit[j];
						outGbestPos[i,1] := outGbestPos[j,1];
						outGbestPos[i,2] := outGbestPos[j,2];
						outGbestPos[i,3] := outGbestPos[j,3];	
					end if;
				end if;
			end if;
		end for;
end for;


end talking;


function acknowledgment "Permette di instaurare una comunicazione tra 2 droni"

InputInt id1,id2;

OutputBool res;

algorithm

res := true;

if(id1 == 3 or id1 == 4) then
	res := false;
	//print("Drone mittente non funzionante\n");
elseif(id2 == 3 or id2 == 4) then
	res := false;
	//print("drone ricevente non funzionante\n");
end if;
end acknowledgment;


function psoComm "Due droni comunicano per scambiare informazioni sull'algoritmo di pathfinding"

//Drone 1

//global fitness
InputReal gBestFit_1;

//Drone 2

InputReal gBestFit_2;


OutputBool gOK;

algorithm

//Se la global fitness del drone 1 è migliore di quella del drone 2, il drone 2 si aggiorna. Altrimenti, il drone 2 manda al drone 1 i suoi valori e il drone 1 deve ripetere la procedura 
gOK := gBestFit_1 >= gBestFit_2;

end psoComm;




