block PSO "Modulo di controllo dell'algoritmo di pathfinding"
	
K const;

//tempo di aggiornamento del pso
parameter Real T = 0.5;

//tendenza dei droni di rimanere alla velocità del precedente timestamp
parameter Real w = 0.6;

//tendenza dei droni di esplorare l'area circostante
parameter Real c1 = 2;

//tendenza dei droni di convergere nel punto di arrivo 
parameter Real c2 = 2;

//Punti di arrivo dei droni
InputReal destX[const.N];
InputReal destY[const.N];
InputReal destZ[const.N];

//Posizione dei droni
InputReal x[const.N];
InputReal y[const.N];
InputReal z[const.N];

//Velocità dei droni
InputReal Vx[const.N];
InputReal Vy[const.N];
InputReal Vz[const.N];

//Stato dei droni
InputInt droneState[const.N];
InputBool droneDead[const.N];
InputBool intrDead[const.nIntr];
InputBool missDead[const.nRocket];

//Droni vicini
InputBool neighbours[const.N,const.N];

//Posizione intrusi
InputReal intrX[const.nIntr];
InputReal intrY[const.nIntr];
InputReal intrZ[const.nIntr];

//posizione missili
InputReal missX[const.nRocket];
InputReal missY[const.nRocket];
InputReal missZ[const.nRocket];

//Posizione ostacoli
InputReal statX[const.nStatObs];
InputReal statY[const.nStatObs];
InputReal statZ[const.nStatObs];


//intrusi vicini a droni
InputBool nearIntr[const.N, const.nIntr];
//Missili vicini a droni
InputBool nearMissile[const.N, const.nRocket];

InputBool nearStatObs[const.N, const.nStatObs];

//Batteria residua di ogni drone
InputReal battery[const.N];

//global fitness e global position sono calcolate e condivise tramite il modulo di comunicazione

//Migliore posizione individuale
Real pBestPos[const.N,3];

//Migliore posizione globale.La gBest pos è unica, ma viene usata sotto forma di vettore per simulare la memorizzazione e la comunicazione della variabile per tutti i droni
Real gBestPos[const.N,3];

//migliore fitness value personale
Real pBestFit[const.N];

//Global fitnessValue. La gBest fit è unica, ma viene usata sotto forma di vettore per simulare la memorizzazione e la comunicazione della variabile per tutti i droni.
//La fitness value è un valore che identifica il livello di ottimizzazione del sistema simulato
Real globFitness[const.N];

//Variabile temporanea per il calcolo e confronto della fitness value
Real tmpFit[const.N];

//valori random per calcolare velocità
Real r1;
Real r2;

//Variabili temporanee per salvataggio dati risultanti dal modulo di comunicazione
Real tmpGPos[const.N,3];
Real tmpGFit[const.N];

//Permette di resettare i valori di fitness, così da aggiornare l'algoritmo
Real timer;

Real tmpBattery[const.N];

OutputReal velocityX[const.N];
OutputReal velocityY[const.N];
OutputReal velocityZ[const.N];

//Scarica batteria dovuta al modulo di comunicazione
OutputReal batteryDischarge[const.N];

initial equation

	for i in 1:const.N loop
		pBestFit[i] = 100000;
		pBestPos[i] = {x[i],y[i],z[i]};

		globFitness[i] = 100000;
		gBestPos[i] = {x[i],y[i],z[i]};
	end for;

	velocityX = zeros(const.N);
	velocityY = zeros(const.N);
	velocityZ = zeros(const.N);
	batteryDischarge = zeros(const.N);

	r1 = 0;
	r2 = 0;
	timer = 0;
	
	tmpGPos = zeros(const.N,3);
	tmpGFit = zeros(const.N);
	tmpFit = zeros(const.N);

algorithm

when sample(0,T) then
	tmpFit := allFitness(const, x,y,z,destX,destY,destZ,intrX,intrY,intrZ, nearIntr, missX, missY, missZ,nearMissile,
	 					statX,statY,statZ,nearStatObs, droneDead, intrDead,missDead);
	if(timer < 2) then
		timer := pre(timer) + 1;
	else 
		timer := 0;
		pBestFit := fill(100000.0, const.N);
		globFitness := fill(100000.0, const.N);
	end if;

	for i in 1:const.N loop
		if(battery[i] > 0 and (not droneDead[i])) then
			//Confronto e setup Pbest. Posso calcolarlo qui poichè richiede solamente i dati del singolo drone.
			if(tmpFit[i] < pBestFit[i]) then
				pBestFit[i] := tmpFit[i];
				pBestPos[i] := {x[i],y[i],z[i]};
			end if;

			//Aggiornamento gBests da inviare al modulo di comunicazione
			if(tmpFit[i] < globFitness[i]) then
				globFitness[i] := tmpFit[i];
				gBestPos[i] := {x[i],y[i],z[i]};
			end if;
			
			r1 := myrandom();
			r2 := myrandom();
			
			velocityX[i] := ((w*Vx[i]) + (c1*r1* (pBestPos[i,1] - x[i])) + (c2*r2* (gBestPos[i,1] - x[i])));
			velocityY[i] := ((w*Vy[i]) + (c1*r1* (pBestPos[i,2] - y[i])) + (c2*r2* (gBestPos[i,2] - y[i])));
			velocityZ[i] := ((w*Vz[i]) + (c1*r1* (pBestPos[i,3] - z[i])) + (c2*r2* (gBestPos[i,3] - z[i])));

			//velocity cap
			(velocityX[i],velocityY[i],velocityZ[i]) := velocityCap(velocityX[i],velocityY[i],velocityZ[i], const.maxSpeed);
			//print("Velocità PSO: (" +String(velocityX[1]) + ", " +String(velocityY[1]) + ", " +String(velocityZ[1]) + ")\n");
		end if;
	end for;

	(tmpGPos, tmpGFit, tmpBattery) := talking(const, globFitness, gBestPos, neighbours, droneState);
	globFitness := tmpGFit;
	gBestPos := tmpGPos;
	batteryDischarge := tmpBattery;
end when;

end PSO;

//Valuta il valore di fitness del singolo drone
function fitness "La fitness value non è altro che la magnitudine tra il drone e la sua destinazione. Se nel percorso si presenta un ostacolo, allora si divide la magnitudine con l'angolo definito tra i 2 vettori"
//posizione droni
InputReal x;
InputReal y;
InputReal z;

InputReal destX, destY, destZ;

//posizione destinazione
InputReal intrX,intrY,intrZ;

OutputReal fitvalue;

	protected 
		Real alpha "Angolo tra i 2 vettori";
		Real dotProd "Prodotto tra 2 punti";
		Real dVector[3] "Vettore tra drone e arrivo";
		Real intrVector[3] "Vettore tra intruso e drone";
		Real fromDtoT "Magnitudine vettore drone-arrivo";
		Real fromDtoIntr "Magnitudine vettore drone-ostacolo";

algorithm
	//Per prima cosa, calcolo l'angolo tra il drone e l'intruso, se presente
	dVector := {destX - x, destY - y, destZ - z};
	intrVector := {intrX - x, intrY-y, intrZ-z};
	dotProd := (dVector[1] * intrVector[1]) + (dVector[2] * intrVector[2]) + (dVector[3] * intrVector[3]); 
	fromDtoT := magnitude(dVector[1], dVector[2], dVector[3]);
	fromDtoIntr := magnitude(intrVector[1],intrVector[2],intrVector[3]);
	if((dotProd / (fromDtoT * fromDtoIntr)) >= 1) then
		fitvalue := 1000*magnitude((destX-x),(destY-y), (destZ-z));
	else
		alpha := acos(dotProd / (fromDtoT * fromDtoIntr)); 
		fitvalue := if(alpha <> 0) then (1000*magnitude((destX-x),(destY-y), (destZ-z)))/ alpha else 1000*magnitude((destX-x),(destY-y), (destZ-z));
	end if;
end fitness;



//La fitness value di un drone viene calcolata basandosi sulla distanza con un ostacolo in movimento (drone nemico, missile, ecc...)
function allFitness

input K const;

//posizione droni
InputReal x[const.N];
InputReal y[const.N];
InputReal z[const.N];

InputReal destX[const.N], destY[const.N], destZ[const.N];

//posizione destinazione
InputReal intrX[const.nIntr];
InputReal intrY[const.nIntr];
InputReal intrZ[const.nIntr];
InputBool nearIntr[const.N, const.nIntr];

//posizione missili
InputReal missX[const.nRocket];
InputReal missY[const.nRocket];
InputReal missZ[const.nRocket];
//Missili vicini a droni
InputBool nearMissile[const.N, const.nRocket];

InputReal statX[const.nStatObs];
InputReal statY[const.nStatObs];
InputReal statZ[const.nStatObs];
InputBool nearStatObs[const.N,const.nStatObs];

//Permette di sapere se droni,ostacoli o missili sono collisi
InputBool droneDead[const.N];
InputBool intrDead[const.nIntr];
InputBool missDead[const.nRocket];

OutputReal fitValue[const.N];

	protected 
		Real tmpFitIntr[const.nIntr], tmpFitMissile[const.nRocket], tmpFitObs[const.nStatObs];

algorithm 
	fitValue := fill(10000000.0,const.N);
	for i in 1:const.N loop
		//Calcolo la fitness value in base alla distanza con gli intrusi
		tmpFitIntr := zeros(const.nIntr);	
		if(not droneDead[i]) then
			for j in 1:const.nIntr loop
				if (nearIntr[i,j] and (not intrDead[j])) then
					tmpFitIntr[j] := fitness(x[i],y[i],z[i], destX[i], destY[i], destZ[i], intrX[j], intrY[j], intrZ[j]);
				else 
					tmpFitIntr[j] := 1000*magnitude((destX[i]-x[i]), (destY[i]-y[i]), (destZ[i]-z[i]));
				end if;
			end for;
			
			for j in 1:const.nIntr loop
				if (tmpFitIntr[j] < fitValue[i]) then
					fitValue[i] := tmpFitIntr[j];
				end if;
			end for;

			//Calcolo la fitness value in base alla distanza con i missili
			tmpFitMissile := zeros(const.nRocket);
			for j in 1:const.nRocket loop
				if (nearMissile[i,j] and (not missDead[j])) then
					tmpFitMissile[j] := fitness(x[i],y[i],z[i], destX[i], destY[i], destZ[i], missX[j], missY[j], missZ[j]);
				else 
					tmpFitMissile[j] := 1000*magnitude((destX[i]-x[i]), (destY[i]-y[i]), (destZ[i]-z[i]));
				end if;
			end for;
			
			for j in 1:const.nRocket loop
				if (tmpFitMissile[j] < fitValue[i]) then
					fitValue[i] := tmpFitMissile[j];
				end if;
			end for;

			//Calcolo la fitness value in base alla distanza con gli ostacoli
			tmpFitObs := zeros(const.nStatObs);
			for j in 1:const.nStatObs loop
				if (nearStatObs[i,j]) then
					tmpFitObs[j] := fitness(x[i],y[i],z[i], destX[i], destY[i], destZ[i], statX[j], statY[j], statZ[j]);
				else 
					tmpFitObs[j] := 1000*magnitude((destX[i]-x[i]), (destY[i]-y[i]), (destZ[i]-z[i]));
				end if;
			end for;
			
			for j in 1:const.nStatObs loop
				if (tmpFitObs[j] < fitValue[i]) then
					fitValue[i] := tmpFitObs[j];
				end if;
			end for;
		end if;
	end for;

end allFitness;



//SEZIONE RELATIVA ALLE FUNZIONI DI COMUNICAZIONE PER IL PSO

function talking "Comunicazione relativa alle info del pso"
	
	input K const;

	InputReal gBestFit[const.N];

	InputReal gBestPos[const.N,3];
	
	//Matrice contenente info sui droni vicini tra loro
	InputBool neighbours[const.N,const.N];

	InputInt droneState[const.N];
	
	OutputReal outGbestPos[const.N,3];

	OutputReal outGbestFit[const.N];

	//Scarica dovuta alla comunicazione tra droni
	OutputReal battery[const.N];

	protected
		Real tmpBatt[const.N];
	
algorithm
outGbestPos := gBestPos;
outGbestFit := gBestFit;
tmpBatt := zeros(const.N);

for i in 1:const.N loop	
		if(droneState[i] <> 4) then //Se il drone ha una fault del sistema di comunicazione, non può scambiare messaggi
			for j in 1:const.N loop
				if(neighbours[i,j] and i <> j) then
					tmpBatt[i] := tmpBatt[i] + 2;
					if(acknowledgment(droneState[i],droneState[j])) then
						tmpBatt[j] := tmpBatt[j] + 1;
						if(psoComm(gBestFit[i], gBestFit[j])) then //Il drone i ha risposto con valori migliori
							outGbestFit[j] := outGbestFit[j];
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
		end if;
end for;

battery := tmpBatt;

end talking;


function acknowledgment "Permette di instaurare una comunicazione tra 2 droni"

InputInt id1,id2;

OutputBool res;

algorithm

res := true;

if(id1 == 4) then
	res := false;
	//print("Drone mittente non funzionante\n");
elseif(id2 == 4) then
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
gOK := gBestFit_1 <= gBestFit_2;

end psoComm;