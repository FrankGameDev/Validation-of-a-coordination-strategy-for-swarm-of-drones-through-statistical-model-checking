record K

//Costanti simulazione

constant Integer N = 10; //Number of uavs

constant Integer nIntr = 5; //Numbers of intruders

constant Integer nRocket = 1;

constant Real g = 9.81; //accellerazione gravitazionale(m/s^2)

constant Real pi = 3.14;

//Dimensione area di volo
constant Real flyZone[3] = {100,100,100};

//--------------------------------------------

//Costanti Drone

//Peso
constant Real m = 0.895;

//Velocità massima di volo(m/s)
constant Real maxSpeed = 15.0;

//Distanza da mantenere tra ogni drone
constant Real dDistance = 1.5; 

//Distanza di Controllo ambientale ad infrarossi dei droni 
constant Real IDD = 8.0;

//Massimo angolo di sterzata
constant Real maxAngle = 30.0;

//Distanza di sicurezza che ogni drone deve mantenere dagli ostacoli(dm) e che utilizza per iniziare la manovra di evasione
constant Real dangerRadius = 5;

//Distanza minima tra droni e ostacolis
constant Real minDistanceFromObs = 2.5;

//Tempo di risposta del sistema utilizzata per anticipare la manovra di collision avoidance
constant Real sysTimeResponse = 0.0025;

//Capcità massima (mAh) di un drone
constant Real capacity = 5000.0;

end K;
