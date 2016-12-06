void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(8,OUTPUT);
}
void loop() {
  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(125);                      
  digitalWrite(LED_BUILTIN, LOW);    
  digitalWrite(8, LOW);
  delay(125);                       
  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(375);                      
  digitalWrite(LED_BUILTIN, LOW);    
   digitalWrite(8, LOW);
  delay(375);                       
 
}
