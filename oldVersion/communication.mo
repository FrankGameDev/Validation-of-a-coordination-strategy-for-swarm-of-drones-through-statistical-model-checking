block Communication "Gestisce la comunicazione tra droni. Prende in input tutti i possibili messaggi e li condivide utilizzando la tecnica di flooding, basata sulla vicinanza dei droni dal drone scelto"

	//Inserire messaggi su Fault

	parameter Real T = 3.0 "Tempo di aggiornamento della comunicazione tra droni";

	//campi appartenenti al PSO
	
	InputReal gBestFit[K.N];

	InputReal gBestPos[K.N,3];
	
	//Matrice contenente info sui droni vicini tra loro
	InputBool neighbours[K.N,K.N];

	InputInt droneState[K.N];

	//InputInt droneState[K.N];

	OutputReal outGbestPos[K.N,3];

	OutputReal outGbestFit[K.N];



initial algorithm

outGbestPos := zeros(K.N,3);
outGbestFit := zeros(K.N);

algorithm

when sample(0,T) then

//aggiungere while finchè tutti i droni non hanno la giusta Gbest
(outGbestPos, outGbestFit) := talking(gBestFit, gBestPos, neighbours);

end when;

end Communication;

function talking
	
	InputReal gBestFit[K.N];

	InputReal gBestPos[K.N,3];
	
	//Matrice contenente info sui droni vicini tra loro
	InputBool neighbours[K.N,K.N];
	
	OutputReal outGbestPos[K.N,3];

	OutputReal outGbestFit[K.N];

	protected
		//memorizza il numero di droni vicini
		Integer cont[K.N];
	
		//memorizza il numero di droni aggiornati dopo lo scambio di messaggi
		Integer numUpdated[K.N];
	
		
	

algorithm
outGbestPos := gBestPos;
outGbestFit := gBestFit;
cont := zeros(K.N);
numUpdated := zeros(K.N);


for i in 1:K.N loop	
	for j in 1:K.N loop
		if(neighbours[i,j] and i<>j) then cont[i] := cont[i] + 1; end if;
	end for;

	//while(numUpdated[i] < cont[i] and cont[i] > 0) loop
		for j in 1:K.N loop
			if(neighbours[i,j] and i <> j) then
				if(acknowledgment(i,j)) then
					if(psoComm(gBestFit[i], gBestFit[j])) then //Il drone j ha risposto con valori migliori
						outGbestFit[j] := outGbestFit[i];
						outGbestPos[j,1] := outGbestPos[i,1];
						outGbestPos[j,2] := outGbestPos[i,2];
						outGbestPos[j,3] := outGbestPos[i,3];
						numUpdated[i] := numUpdated[i] +1;
					else
						outGbestFit[i] := outGbestFit[j];
						outGbestPos[i,1] := outGbestPos[j,1];
						outGbestPos[i,2] := outGbestPos[j,2];
						outGbestPos[i,3] := outGbestPos[j,3];	
						numUpdated[i] := 0;					
					end if;
				end if;
			end if;
		end for;
	//end while;
end for;

for i in 1:K.N loop
print(String(numUpdated[i]));
print(" i = " + String(i) + " --> Fit: " + String(outGbestFit[i])+ " x: " + String(outGbestPos[i,1]) + " y: " + String(outGbestPos[i,2]) + " z: " + String(outGbestPos[i,3])+"\n"); 
end for;


end talking;


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


OutputBool gOK;

algorithm

//Se la global fitness del drone 1 è migliore di quella del drone 2, il drone 2 si aggiorna. Altrimenti, il drone 2 manda al drone 1 i suoi valori e il drone 1 deve ripetere la procedura 
gOK := gBestFit_1 <= gBestFit_2;

end psoComm;
