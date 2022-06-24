block faultSys "Gestisce le possibili fault dei droni. Possono esserci fault alla sensoristica, alle componenti di manovra e/o alle componenti di comunicazione"

/*
In caso di:
- Fault della sensoristica: il modulo di collisionAvoidance non riuscirà a calcolare la traiettoria esatta al fine di non far collidere 2 droni
- Fault del modulo di comunicazione: Le informazioni riguardanti l'algoritmo di pathfinding non vengono condivise con il drone soggetto al fault e con i suoi vicini
- Fault di manovra: Il drone smette di funzionare

*/
	K const;

	parameter Real T = 2.5 "Tempo di aggiornamento probabilità fault";	

/* 	//Matrice che identifica la probabilità di transizione
	//Riga 1 = funzionante; Riga 2 = sensoristica; Riga 3 = manovra; Riga 4 = comunicazione
	parameter Real transMatrix[4,4] = [0.7, 0.1, 0.1, 0.1;
										0.6, 0.4, 0, 0;
										0.5, 0, 0.5, 0;
										0.7, 0, 0, 0.3]; */
									
	parameter Real transMatrix[4,4] = {{1, 0, 0, 0},
										{1, 0, 0, 0},
										{1, 0, 0, 0},
										{1, 0, 0, 0}};

	//probabilità calcolatà randomicamente
	Real prob;

	//stato del drone. Viene restituito al drone stesso per valutare la sua situazione
	OutputInt state[const.N];
	
algorithm
	//inizializzo lo stato 
	when initial() then
		state := fill(1,const.N);
		prob := 0;

	elsewhen sample(0,T) then
		for i in 1:const.N loop	
			prob := myrandom();
			//print("Drone "+ String(i) +": " + String(state[i])+", " + String(prob) + "\n");
			state[i] := nextState(pre(state[i]), transMatrix, prob);
		end for;
	end when;

end faultSys;

function nextState "calcola il nuovo stato del drone. Nel caso in cui il valore del random sia maggiore della probabilità in posizione 1 dello stato di partenza, allora aggiorniamo val finchè val >= z"
	
	InputInt state;
	InputReal matr[:,:];
	InputReal rand;	

	OutputInt newState;

	protected 
		Integer i;
		Real val;
algorithm
	i := 1;
	val := matr[state,i];
	while(i < size(matr, 1) and (rand > val)) loop
		i := i+1;		
		val := val + matr[state,i];
	end while; 

	newState := i;
	

end nextState;



 
