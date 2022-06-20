block CollisionManagement"Controlla se i droni collidono con oggetti nell'area di volo"
	K const;
	parameter Real T = 1.5; //Refresh controllo collisione

	//input Vector3D drones[const.N];

	InputReal x[const.N];
	InputReal y[const.N];
	InputReal z[const.N];

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

	Real tDD, tDC, tDR, tDSC;

    //Vettori che memorizzano informazioni relative alle collisioni
    OutputBool droneDead[const.N];
    OutputBool intrDead[const.nIntr];
    OutputBool missDead[const.nRocket];

    OutputBool collision;

initial algorithm
	tDD := 0;
	tDC := 0;
	tDR := 0;
    tDSC := 0;
    droneDead := fill(false,const.N);
    intrDead := fill(false,const.nIntr);
    missDead := fill(false,const.nRocket);
    collision := false;
algorithm

when sample(0,T) then		
	//Dato un drone, controlla se collide con gli altri k-1 droni
	for i in 1:const.N loop //drone da controllare
        if(not droneDead[i]) then
            for j in 1:const.N loop //droni nelle vicinanze 		
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
		
            for j in 1:const.nIntr loop//Controlla le collisioni con gli intrusi
                if((not intrDead[j]) and checkPosition(x[i],y[i],z[i],intrX[j], intrY[j], intrZ[j])) then
                    print("Collisione tra drone e intruso(" + String(i) + ", " + String(j) + ")\n");
                    tDC := tDC + 1;
                    droneDead[i] := true;
                    intrDead[j] := true;
                    end if;
            end for;

            for j in 1:const.nRocket loop//Controlla le collisioni con gli intrusi
                if((not missDead[j]) and checkPosition(x[i],y[i],z[i],missX[j], missY[j], missZ[j])) then
                    print("Collisione tra drone e missile(" + String(i) + ", " + String(j) + ")\n");
                    tDR := tDR + 1; 
                    droneDead[i] := true;
                    missDead[j] := true;
                end if;
            end for;

            for j in 1:const.nStatObs loop//Controlla le collisioni con gli intrusi
                if(checkPosition(x[i],y[i],z[i],statX[j], statY[j], statZ[j])) then
                    print("Collisione tra drone e ostacolo fisso(" + String(i) + ", " + String(j) + ")\n");
                    tDSC := tDSC + 1;
                    droneDead[i] := true;
                    end if;
            end for;
        end if;
	end for;

    if(tDC > 0 or tDD > 0 or tDR > 0 or tDSC > 0) then
        collision := true;
    end if;
end when;


end CollisionManagement;


function checkPosition

	InputReal x,y,z;
	InputReal x2,y2,z2;

	OutputBool col;

algorithm
	col := if(euclideanDistance(x,y,z,x2,y2,z2) < 1.5) then true else false;
end checkPosition;



				
