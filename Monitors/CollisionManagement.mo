block CollisionManagement"Controlla se i droni collidono con oggetti nell'area di volo"
	
	parameter Real T = 1; //Refresh controllo collisione

	//input Vector3D drones[K.N];

	InputReal x[K.N];
	InputReal y[K.N];
	InputReal z[K.N];

	InputReal intrX[K.nIntr];
	InputReal intrY[K.nIntr];
	InputReal intrZ[K.nIntr];

	//Posizione missili
	InputReal missX[K.nRocket];
	InputReal missY[K.nRocket];
	InputReal missZ[K.nRocket];

    //Posizione ostacoli
	InputReal statX[K.nStatObs];
	InputReal statY[K.nStatObs];
	InputReal statZ[K.nStatObs];

	Real tDD, tDC, tDR, tDSC;

    //Vettori che memorizzano informazioni relative alle collisioni
    OutputBool droneDead[K.N];
    OutputBool intrDead[K.nIntr];
    OutputBool missDead[K.nRocket];


initial algorithm
	tDD := 0;
	tDC := 0;
	tDR := 0;
    tDSC := 0;
    droneDead := fill(false,K.N);
    intrDead := fill(false,K.nIntr);
    missDead := fill(false,K.nRocket);

algorithm

when sample(0,T) then		
	//Dato un drone, controlla se collide con gli altri k-1 droni
	for i in 1:K.N loop //drone da controllare
        if(not droneDead[i]) then
            for j in 1:K.N loop //droni nelle vicinanze 		
                if(i <> j and (not droneDead[j])) then 			
                    //se 2 droni si trovano nella stessa posizione, collision diventa true
                    if(checkPosition(x[i],y[i],z[i],x[j],y[j],z[j])) then
                        tDD := tDD + 1;
                        print("Collisione tra droni (" + String(i) + ", " + String(j) + ")\n");
                        droneDead[i] := true;
                        droneDead[j] := true;
                    end if;
                end if;
		    end for;
		
            for j in 1:K.nIntr loop//Controlla le collisioni con gli intrusi
                if((not intrDead[j]) and checkPosition(x[i],y[i],z[i],intrX[j], intrY[j], intrZ[j])) then
                    print("Collisione tra drone e intruso(" + String(i) + ", " + String(j) + ")\n");
                    tDC := tDC + 1;
                    droneDead[i] := true;
                    intrDead[j] := true;
                    end if;
            end for;

            for j in 1:K.nRocket loop//Controlla le collisioni con gli intrusi
                if((not missDead[j]) and checkPosition(x[i],y[i],z[i],missX[j], missY[j], missZ[j])) then
                    print("Collisione tra drone e missile(" + String(i) + ", " + String(j) + ")\n");
                    tDR := tDR + 1; 
                    droneDead[i] := true;
                    missDead[j] := true;
                end if;
            end for;

            for j in 1:K.nStatObs loop//Controlla le collisioni con gli intrusi
                if(checkPosition(x[i],y[i],z[i],statX[j], statY[j], statZ[j])) then
                    print("Collisione tra drone e ostacolo fisso(" + String(i) + ", " + String(j) + ")\n");
                    tDSC := tDSC + 1;
                    droneDead[i] := true;
                    end if;
            end for;
        end if;
	end for;
end when;

end CollisionManagement;


function checkPosition

	InputReal x,y,z;
	InputReal x2,y2,z2;

	OutputBool col;

algorithm
	col := false;
	if(euclideanDistance(x,y,z,x2,y2,z2) < 2) then
		col := true;
	end if;
	
end checkPosition;



				
