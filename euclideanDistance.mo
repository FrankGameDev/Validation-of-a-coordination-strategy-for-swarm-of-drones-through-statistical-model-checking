function euclideanDistance
	
	//Drone 1
	InputReal x1;
	InputReal y1;
	InputReal z1;
	
	//Drone 2
	InputReal x2;
	InputReal y2;
	InputReal z2;

	OutputReal dist; 

algorithm
	
	dist := sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2);	
	

end euclideanDistance;
