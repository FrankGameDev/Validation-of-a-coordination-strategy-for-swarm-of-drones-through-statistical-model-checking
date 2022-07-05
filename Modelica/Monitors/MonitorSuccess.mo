block MonitorSuccess "Monitora il numero di droni arrivati a destinazione e il loro tempo di arrivo"

    parameter Real T = 1 "Tempo di aggiornamento controllo monitor";
    
    K const;
    
    //Posizione droni
	InputReal x[const.N], y[const.N], z[const.N];
    
    //Destinazione droni
    InputReal destX[const.N], destY[const.N], destZ[const.N];

    //Batteria droni
    InputReal batterySensDischarge[const.N], batteryPSODischarge[const.N];

    //Stato del drone
    InputBool droneDead[const.N];

    //Tempo di arrivo dei droni. Se -1, drone morto
    OutputReal arrivalTime[const.N];

    //Identifica se i droni sono arrivati o meno a destinazione
    OutputBool arrived[const.N];

    Real battery;
    Boolean timeToEnd;


initial equation
    arrivalTime = fill(0, const.N);
    arrived = fill(false,const.N);
    timeToEnd = false;
algorithm
    when sample(0,T) then
        for i in 1:const.N loop
            battery := batterySensDischarge[i] - batteryPSODischarge[i];
            if(battery > 0 and (not droneDead[i])) then
                arrived[i] := if(not arrived[i]) then checkArrived(x[i],y[i],z[i],destX[i],destY[i],destZ[i]) else true;
                if(arrivalTime[i] <= 0 and arrived[i]) then 
                    arrivalTime[i] := time;
                    // print("drone " +String(i)+ " arrivato e Tempo di arrivo: (" + String(arrived[i]) + ", " + String(arrivalTime[i]) + ")\n");
                end if;
            else 
                arrivalTime[i] := -1;
            end if;
        end for;    
        
        timeToEnd := true;
        for i in 1:const.N loop
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

    protected 
        K const;

algorithm
    arrived := if(euclideanDistance(x,y,z,destX,destY,destZ) <= const.arrivalThreshold) then true else false;
end checkArrived;