record K

//Costanti simulazione

parameter Integer N = 4; //Number of uavs

parameter Integer nIntr = 10; //Number of intruders

parameter Integer nRocket = 10; //Number of missile

parameter Integer nStatObs = 8; //Number of static obstacle

parameter Real g = 9.81; //accellerazione gravitazionale(m/s^2)

parameter Real pi = 3.14;

//Costante utile alla cattura dei risultati. Valuta una distanza massima dal punto di arrivo al fine di considerare il drone arrivato a destinazione
parameter Real arrivalThreshold = 5;

//Dimensione area di volo
parameter Real flyZone[3] = {200,200,200};

//Distanza minima da percorrere dal punto di partenza a quello di arrivo
parameter Real minDestDistance = flyZone[1]/2;

//--------------------------------------------

//Costanti Drone

//Peso
parameter Real m = 0.895;

//Velocità massima di volo(m/s)
parameter Real maxSpeed = 5;

//Distanza da mantenere tra ogni drone
parameter Real dDistance = 25; 

//Distanza di Controllo ambientale ad infrarossi dei droni 
parameter Real IDD = 8.0;

//Distanza di controllo ambientale tramite videocamera
//Orizzontale
parameter Real horizontalODD = 40;
//Verticale
parameter Real verticalODD = 30;

//Massimo angolo di sterzata
parameter Real maxAngle = 30.0;

//Distanza di sicurezza che ogni drone deve mantenere dagli ostacoli(dm) e che utilizza per iniziare la manovra di evasione
parameter Real dangerRadius = 8.0;

//Distanza minima tra droni e ostacolis
parameter Real minDistanceFromObs = 5;

//Capcità massima (mAh) di un drone
parameter Real capacity = 5000.0;

// ----------------------------

//Costanti intrusi

parameter Real intrudersMass = 0.7;

// ---------------------------

//Costanti missili

//Raggio di rilevamento droni
parameter Real detectionDistance = 50.0;

parameter Real rocketMass = 1.5;

end K;
