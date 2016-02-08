void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
}

void loop() {
   int light1 = analogRead(A3); //front sensor
   int light2 = analogRead(A4); //left sensor
   int light3 = analogRead(A5); //back sensor
   int light4 = analogRead(A6); //right sensor
   int lightsensors[5]={light1,light2,light3,light4};
   int distance1 = analogRead(A2);
   int distance2 = analogRead(A7);
   int distancesensors[3]={distance1,distance2};
  Serial.print("light sensors:");
int i;
for (i = 0; i < 4; i = i + 1) {
  Serial.print(lightsensors[i]);
  Serial.print(", ");
}
 Serial.print("   distance sensors:");

 for (i = 0; i < 2; i = i + 1) {
  Serial.print(distancesensors[i]);
  Serial.print(", ");
}
 Serial.println(" ");
 delay(2000); 
  // put your main code here, to run repeatedly:
}


  
  


