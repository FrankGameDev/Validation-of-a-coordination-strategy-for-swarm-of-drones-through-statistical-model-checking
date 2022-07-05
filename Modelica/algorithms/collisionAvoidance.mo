block CollisionAvoidance 
"Modulo si occupa di prevenire collisioni con gli ostacoli. Deve quindi rilevare ostacoli nella traievectBetweenoria, 
mantenere la distanza di sicurezza e decidere una nuova traievectBetweenoria se l'ostacolo si trova troppo vicino"
K const;
parameter Real T = 0.5;

//Punti di arrivo dei droni
InputReal destX[const.N];
InputReal destY[const.N];
InputReal destZ[const.N];

//Posizione dei droni
InputReal x[const.N];
InputReal y[const.N];
InputReal z[const.N];

//Stato dei droni
InputInt droneState[const.N];

//Posizione intrusi
InputReal intrX[const.nIntr];
InputReal intrY[const.nIntr];
InputReal intrZ[const.nIntr];

//Posizione missili
InputReal missX[const.nRocket];
InputReal missY[const.nRocket];
InputReal missZ[const.nRocket];

//Posizione ostacoli
InputReal statX[const.nStatObs];
InputReal statY[const.nStatObs];
InputReal statZ[const.nStatObs];

InputBool nearIntr[const.N,const.nIntr];
InputBool nearMissile[const.N, const.nRocket];
InputBool nearStatObs[const.N, const.nStatObs];
InputReal battery[const.N];

//Permette di sapere se droni,ostacoli o missili sono collisi
InputBool droneDead[const.N];
InputBool intrDead[const.nIntr];
InputBool missDead[const.nRocket];

//Nuove dest
OutputReal tmpDestX[const.N], tmpDestY[const.N], tmpDestZ[const.N];
//Permette al controller del drone di utilizzare la nuova destinazione per il calcolo del trust
OutputBool useTMPDest[const.N];

initial equation
    tmpDestX = destX;
    tmpDestY = destY;
    tmpDestZ = destZ;
    useTMPDest = fill(false, const.N);

algorithm
when sample(1,T) then
    for i in 1:const.N loop
        if(battery[i] > 0 and droneState[i] <> 2 and (not droneDead[i])) then
            useTMPDest[i] := false;
            //1) Controllo collision avoidance per ostacoli
            for j in 1:const.nIntr loop 
                if(nearIntr[i,j] and (not intrDead[j])) then
                    //2): controllo se il drone è a distanza di sicurezza dall'ostacolo
                    if(euclideanDistance(x[i], y[i], z[i], intrX[j], intrY[j], intrZ[j]) <= const.dangerRadius) then
                        (tmpDestX[i], tmpDestY[i], tmpDestZ[i]) := findNewDestination(x[i], y[i], z[i], destX[i], destY[i], destZ[i], intrX[j], intrY[j], intrZ[j]); 
                        useTMPDest[i] := true; 
                    end if;
                end if;
            end for;

            //1) Controllo collision avoidance per missili
            for j in 1:const.nRocket loop 
                if(nearMissile[i,j] and (not missDead[j])) then
                    if(euclideanDistance(x[i], y[i], z[i], missX[j], missY[j], missZ[j]) <= const.dangerRadius) then
                        (tmpDestX[i], tmpDestY[i], tmpDestZ[i]) := findNewDestination(x[i], y[i], z[i], destX[i], destY[i], destZ[i], missX[j], missY[j], missZ[j]); 
                        useTMPDest[i] := true; 
                    end if;
                end if;
            end for;
            
            //2) Controllo collision avoidance per ostacoli
            for j in 1:const.nStatObs loop 
                if(nearStatObs[i,j]) then
                    if(euclideanDistance(x[i], y[i], z[i], statX[j], statY[j], statZ[j]) <= const.dangerRadius) then
                        (tmpDestX[i], tmpDestY[i], tmpDestZ[i]) := findNewDestination(x[i], y[i], z[i], destX[i], destY[i], destZ[i], statX[j], statY[j], statZ[j]); 
                        useTMPDest[i] := true; 
                    end if;
                end if;
            end for;
        end if;
    end for;


end when;

end CollisionAvoidance;

/*
Una volta rilevata una minaccia, valuto una nuova destinazione in base al 
vettore tra il drone e l'ostacolo.
Se x > 0, allora la nuova x del vettore destinazione sarà  destX - x - dangerRadius.
Se x < 0, allora destX + x + dangerRadius.
Lo stesso vale per la y e la z.
La modifica orizzontale e verticale viene valutata solamente se i valori di entrambi risultano essere
< del dangerRadius
*/                 
function findNewDestination
    //Posizione dei droni
    InputReal x;
    InputReal y;
    InputReal z;

    //Punti di arrivo dei droni
    InputReal destX;
    InputReal destY;
    InputReal destZ;

    //Posizione intrusi
    InputReal intrX;
    InputReal intrY;
    InputReal intrZ;

    //Nuova destinazione temporanea del drone
    OutputReal newDestX, newDestY, newDestZ;

    protected   
        //Vettore tmp tra posizione drone e ostacolo
        Real vectDtoC[3];
        K const;


algorithm   
    //1) Calcolo il vettore tra l'ostacolo e il drone
    vectDtoC := {intrX - x, intrY - y, intrZ - z};
    //2) Inizializzo l'output  uguale alla vecchia destinazione
    newDestX := destX;
    newDestY := destY;
    newDestZ := destZ;

    //3) Controllo se il nuovo vettore rientra nel dangerous radius del drone. 
    //   Controllo x,y e z separatamente, cosi da modificare la destinazione correttamente.

    //Controllo x
    if(abs(vectDtoC[1]) < const.dangerRadius) then
        if(vectDtoC[1] >= 0) then 
            newDestX := destX - vectDtoC[1] - const.dangerRadius; 
        else
            newDestX := destX + vectDtoC[1] + const.dangerRadius; 
        end if;
    end if;
    
    //Controllo y
    if(abs(vectDtoC[2]) < const.dangerRadius) then
        if(vectDtoC[2] >= 0) then 
            newDestY := destY - vectDtoC[2] - const.dangerRadius; 
        else
            newDestY := destY + vectDtoC[2] + const.dangerRadius; 
        end if;
    end if;
    
    //Controllo z
    if(abs(vectDtoC[3]) < const.dangerRadius) then
        if(vectDtoC[3] >= 0) then 
            if(z > const.dangerRadius + vectDtoC[3]) then //Controllo effettuato se il drone si trova troppo vicino al terreno
                newDestZ := destZ - vectDtoC[3] - const.dangerRadius; 
            end if;
        else
            newDestZ := destZ + vectDtoC[3] + const.dangerRadius; 
        end if;
    end if;

end findNewDestination;