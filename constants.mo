record K

//Costanti simulazione

constant Integer N = 2; //Number of uavs

constant Integer nIntr = 5; //Number of intruders

constant Integer nRocket = 5; //Number of missile

constant Integer nStatObs = 5; //Number of static obstacle

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
constant Real dDistance = 15; 

//Distanza di Controllo ambientale ad infrarossi dei droni 
constant Real IDD = 8.0;

//Distanza di controllo ambientale tramite videocamera
//Orizzontale
constant Real horizontalODD = 40;
//Verticale
constant Real verticalODD = 30;

//Massimo angolo di sterzata
constant Real maxAngle = 30.0;

//Distanza di sicurezza che ogni drone deve mantenere dagli ostacoli(dm) e che utilizza per iniziare la manovra di evasione
constant Real dangerRadius = 8.0;

//Distanza minima tra droni e ostacolis
constant Real minDistanceFromObs = 3;

//Capcità massima (mAh) di un drone
constant Real capacity = 5000.0;

//Costanti intrusi

constant Real intrudersMass = 0.7;

//Costanti missili

//Raggio di rilevamento droni
constant Real detectionDistance = 50.0;

constant Real rocketMass = 1.5;

end K;
