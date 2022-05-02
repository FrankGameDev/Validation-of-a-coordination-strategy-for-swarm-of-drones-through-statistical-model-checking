block MonitorSuccess "Monitora il numero di droni arrivati a destinazione e il loro tempo di arrivo"

    parameter Real T = 1 "Tempo di aggiornamento controllo monitor";

    //Posizione droni
	InputReal x[K.N], y[K.N], z[K.N];
    
    //Destinazione droni
    InputReal destX[K.N], destY[K.N], destZ[K.N];

    //Batteria droni
    InputReal batterySensDischarge[K.N], batteryPSODischarge[K.N];

    //Stato del drone
    InputBool droneDead[K.N];

    //Tempo di arrivo dei droni. Se -1, drone morto
    OutputReal arrivalTime[K.N];

    //Identifica se i droni sono arrivati o meno a destinazione
    OutputBool arrived[K.N];

    Real battery;
    Boolean timeToEnd;

initial equation
    arrivalTime = fill(0, K.N);
    arrived = fill(false,K.N);
    timeToEnd = false;
algorithm
    when sample(0,T) then
        for i in 1:K.N loop
            battery := batterySensDischarge[i] - batteryPSODischarge[i];
            if(battery > 0 and (not droneDead[i])) then
                arrived[i] := if(not arrived[i]) then checkArrived(x[i],y[i],z[i],destX[i],destY[i],destZ[i]) else true;
                if(arrivalTime[i] <= 0 and arrived[i]) then 
                    arrivalTime[i] := time;
                    print("drone " +String(i)+ " arrivato e Tempo di arrivo: (" + String(arrived[i]) + ", " + String(arrivalTime[i]) + ")\n");
                end if;
            else 
                arrivalTime[i] := -1;
            end if;
        end for;    
        
        timeToEnd := true;
        for i in 1:K.N loop
            if((not arrived[i]) and arrivalTime[i] >= 0) then
                timeToEnd := false;
            end if;
        end for;
        if(timeToEnd) then terminate("Tutti i droni hanno raggiunto la destinazione oppure hanno avuto collisioni"); end if;

    end when;

end MonitorSuccess;

function checkArrived

    //Posizione droni
	InputReal x, y, z;
    
    //Destinazione droni
    InputReal destX, destY, destZ;

    OutputBool arrived;

algorithm
    arrived := if(euclideanDistance(x,y,z,destX,destY,destZ) <= K.arrivalThreshold) then true else false;
end checkArrived;