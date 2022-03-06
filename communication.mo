block Communication "Gestisce la comunicazione tra droni. Prende in input tutti i possibili messaggi e li condivide utilizzando la tecnica di flooding, basata sulla vicinanza dei droni dal drone scelto"

	//Inserire messaggi su Fault

	parameter Real T = 3.0 "Tempo di aggiornamento della comunicazione tra droni";

	//campi appartenenti al PSO
	
	InputReal gBestFit[K.N];

	InputReal gBestPos[K.N,3];
	
	//Matrice contenente info sui droni vicini tra loro
	InputBool neighbours[K.N,K.N];

	//fitness calcolata per ogni drone
	InputReal tmpFit[K.N];

	OutputReal outGbestPos[K.N,3];

	OutputReal outGbestFit[K.N];

	//Salva informazioni sull'eventuale aggiornamento di Gbest per il drone che sta comunicando
	Integer isGBestUp;

	//Salvo temporanemante informazioni sull'aggiornamento delle variabili del PSO. True se aggiornate correttamente, altrimenti False
	Integer updatedDrones[K.N];

	Integer cont;	


algorithm

when initial() then

isGBestUp := 0;
updatedDrones := zeros(K.N);

outGbestPos := zeros(K.N,3);
outGbestFit := zeros(K.N);

end when;

when sample(0,T) then



for i in 1:K.N loop	
	updatedDrones := zeros(K.N);
	cont := 0;
	for j in 1:K.N loop
		if(neighbours[i,j]) then cont := cont+1; end if;
	end for;
	//print(String(outGbestPos[i,1]) + "," + String(outGbestPos[i,2]) + ", " + String(outGbestPos[i,3])+"\n");
//Questo while permette di ripetere l'invio dei dati se i droni "j" hanno valori migliori rispetto al drone "i"		
	isGBestUp := 0; 	
	while(isGBestUp < cont) loop		
		for j in 1:K.N loop
			if(neighbours[i,j] and updatedDrones[j]==0) then
				if(acknowledgment(i,j)) then
					if(not psoComm(gBestFit[i], gBestFit[j], tmpFit[j])) then //Il drone j ha risposto con valori migliori
						outGbestFit[i] := gBestFit[j];
						outGbestPos[i] := gBestPos[j];
						updatedDrones := zeros(K.N);
					else
						outGbestFit[j] := gBestFit[i];
						outGbestPos[j] := gBestPos[i];
						updatedDrones[j] := 1;
						isGBestUp := isGBestUp +1;						
					end if;
				end if;
			end if;
		end for;
	end while;
end for;

end when;

end Communication;


function acknowledgment "Permette di instaurare una comunicazione tra 2 droni"

InputInt id1,id2;

OutputBool res;

algorithm

//if(ID2 ha fault e non può comunicare) then false else true end if;
res := true;

end acknowledgment;


function psoComm "Due droni comunicano per scambiare informazioni sull'algoritmo di pathfinding"

//Drone 1

//global fitness
InputReal gBestFit_1;

//Drone 2

InputReal gBestFit_2;

//fitness value del drone 2
InputReal tmpFit_2;


OutputBool gOK;

algorithm

//Se la global fitness del drone 1 è migliore di quella del drone 2, il drone 2 si aggiorna. Altrimenti, il drone 2 manda al drone 1 i suoi valori e il drone 1 deve ripetere la procedura 
if(gBestFit_1 <= gBestFit_2) then
	gOK := true;
else
	gOK := false;

end if;

end psoComm;
