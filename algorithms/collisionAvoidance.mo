block CollisionAvoidance 
"Modulo si occupa di prevenire collisioni con gli ostacoli. Deve quindi rilevare ostacoli nella traievectBetweenoria, 
mantenere la distanza di sicurezza e decidere una nuova traievectBetweenoria se l'ostacolo si trova troppo vicino"

parameter Real T = 1;

//Punti di arrivo dei droni
InputReal destX[K.N];
InputReal destY[K.N];
InputReal destZ[K.N];

//Posizione dei droni
InputReal x[K.N];
InputReal y[K.N];
InputReal z[K.N];

//Velocità dei droni
InputReal Vx[K.N];
InputReal Vy[K.N];
InputReal Vz[K.N];

//Stato dei droni
InputInt droneState[K.N];

//Posizione intrusi
InputReal intrX[K.nIntr];
InputReal intrY[K.nIntr];
InputReal intrZ[K.nIntr];

//velocità intrusi
InputReal vIntrX[K.nIntr];
InputReal vIntrY[K.nIntr];
InputReal vIntrZ[K.nIntr];

InputBool nearIntr[K.N,K.nIntr];

//Nuove dest
OutputReal tmpDestX[K.N], tmpDestY[K.N], tmpDestZ[K.N];
//Permette al controller del drone di utilizzare la nuova destinazione per il calcolo del trust
OutputBool useTMPDest[K.N];

initial equation
    tmpDestX = destX;
    tmpDestY = destY;
    tmpDestZ = destZ;
    useTMPDest = fill(false, K.N);

algorithm
when sample(1,T) then
    for i in 1:K.N loop
        useTMPDest[i] := false;
        for j in 1:K.nIntr loop 
            if(nearIntr[i,j]) then
                //Passo 1: controllo se il drone è a distanza di sicurezza dall'ostacolo
                if(euclideanDistance(x[i], y[i], z[i], intrX[j], intrY[j], intrZ[j]) <= K.dangerRadius) then
                    (tmpDestX[i], tmpDestY[i], tmpDestZ[i]):= findNewDestination(x[i], y[i], z[i], destX[i], destY[i], destZ[i], intrX[j], intrY[j], intrZ[j]); 
                    useTMPDest[i] := true; 
                    print("i: " + String(i) + "\n");
                    print("Vecchia destinazione: (" + String(destX[i]) + ", " + String(destY[i]) + ", " 
                    +  String(destZ[i]) + ");\n");
                    print("Nuova destinazione: (" + String(tmpDestX[i]) + ", " + String(tmpDestY[i]) + ", " 
                    +  String(tmpDestZ[i]) + ");\n");
                end if;
            end if;
        end for;
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
    if(abs(vectDtoC[1]) < K.dangerRadius) then
        if(vectDtoC[1] >= 0) then 
            newDestX := destX - vectDtoC[1] - K.dangerRadius; 
        else
            newDestX := destX + vectDtoC[1] + K.dangerRadius; 
        end if;
    end if;
    
    //Controllo y
    if(abs(vectDtoC[2]) < K.dangerRadius) then
        if(vectDtoC[2] >= 0) then 
            newDestY := destY - vectDtoC[2] - K.dangerRadius; 
        else
            newDestY := destY + vectDtoC[2] + K.dangerRadius; 
        end if;
    end if;
    
    //Controllo z
    if(abs(vectDtoC[3]) < K.dangerRadius) then
        if(vectDtoC[3] >= 0) then 
            newDestZ := destZ - vectDtoC[3] - K.dangerRadius; 
        else
            newDestZ := destZ + vectDtoC[3] + K.dangerRadius; 
        end if;
    end if;

end findNewDestination;


/*
function avoidanceTime

    //Posizione dei droni
    InputReal x;
    InputReal y;
    InputReal z;

    //Velocità dei droni
    InputReal Vx;
    InputReal Vy;
    InputReal Vz;

    //Posizione intrusi
    InputReal intrX;
    InputReal intrY;
    InputReal intrZ;


    //velocità intrusi
    InputReal vIntrX;
    InputReal vIntrY;
    InputReal vIntrZ;

    OutputReal critTime_XY, critTime_XZ;


    protected 
        Real velRelative[3];
        //distanza tra drone e ostacolo
        Real r_XY, r_XZ;
        //angolo tra velocità drone e velocità ostacolo
        Real thetaVr_XY, thetaVr_XZ;
        //angolo vevectBetweenore distanza
        Real thetaR_XY, thetaR_XZ;
        //angolo tra drone e ostacolo
        Real thetaT_XY, thetaT_XZ;   

algorithm
    //1)Calcolo la velocità relativa tra drone e intruso, il vevectBetweenore tra i 2 e la direzione
        velRelative[1] := vIntrX - Vx; 
        velRelative[2] := vIntrY - Vy; 
        velRelative[3] := vIntrZ - Vz; 
        r_XY := euclideanDistance2D(x,y,intrX,intrY);
        r_XZ := euclideanDistance2D(x,z,intrX,intrZ);

    //2) Calcolo gli angoli sul XY(orizzontale) e XZ(verticale)
        //XY
        thetaVr_XY := vectorAngle2D(Vx, Vy, vIntrX, vIntrY);
        thetaR_XY := vectorAngle2D(x, y, intrX, intrY);
        thetaT_XY := thetaVr_XY - thetaR_XY;
        //XZ
        thetaVr_XZ := vectorAngle2D(Vx, Vz, vIntrX, vIntrZ);
        thetaR_XZ := vectorAngle2D(x, z, intrX, intrZ);
        thetaT_XZ := thetaVr_XZ - thetaR_XZ;

    //3)Calcolo il tempo di schivata critico su XY(orizzontale) e XZ(verticale)
        critTime_XY := (r_XY*cos(thetaT_XY) - 2*K.dangerRadius + r_XY*sin(thetaT_XY) / magnitude2D(velRelative[1],velRelative[2])) - K.sysTimeResponse;
        critTime_XZ := (r_XZ*cos(thetaT_XZ) - 2*K.dangerRadius + r_XZ*sin(thetaT_XZ) / magnitude2D(velRelative[1],velRelative[3])) - K.sysTimeResponse;
        //print("Time xy = " + String(critTime_XY) + ", Time XZ = " + String(critTime_XZ) + "\n");
end avoidanceTime;
*/