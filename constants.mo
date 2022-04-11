record K

constant Integer N = 10; //Number of uavs

constant Integer nIntr = 20; //Numbers of intruders

constant Real g = 9.81; //accellerazione gravitazionale(m/s^2)

constant Real m = 0.895;

//Velocità massima di volo senza vento (m/s)
constant Real maxSpeed = 15.0;

//Distanza da mantenere tra ogni drone
constant Real dDistance = 1.5; 

//Distanza di Controllo ambientale ad infrarossi dei droni 
constant Real IDD = 8.0;

//Dimensione area di volo
constant Real flyZone[3] = {100,100,100};

end K;
