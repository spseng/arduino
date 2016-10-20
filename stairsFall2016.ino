#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

  int notearray[8];
  int oldnotearray[8];
  int boolarray[8];
  
  int notefrequencies[] = {
  NOTE_C4, 
  NOTE_D4, 
  NOTE_E4, 
  NOTE_F4,
  NOTE_G4,
  NOTE_A4,
  NOTE_B4,
  NOTE_C5
  };
  
void setup() {
  analogReference(INTERNAL2V56);
   Serial.begin(9600);      // open the serial port at 9600 bps:    

  
  //
}

void initarray()
{
  for (int i=0; i<8; i++)
    notearray[i]=0;
}

void displayarray()
{
  for (int i=0; i<8; i++)
  {
    Serial.print(boolarray[i]);
  }
  Serial.print("\n");
}

void updatearray()
{
 // initarray();
  
  for (int i=0; i<8; i++)
  {
    oldnotearray[i]=notearray[i];
  }
  notearray[0]= analogRead(A0);
  notearray[1]= analogRead(A1);
  notearray[2]= analogRead(A2);
  notearray[3]= analogRead(A3);
  notearray[4]= analogRead(A4);
  notearray[5]= analogRead(A5);
  notearray[6]= analogRead(A6);
  notearray[7]= analogRead(A7);

  
  
  for (int i=0; i<8; i++)
  {
    if (notearray[i]>200 && oldnotearray[i]>200) 
    {
      boolarray[i]=1; 
    }
    else 
    {
      boolarray[i]=0; 
    } 
  }
  
}

void loop()
{
 
  int noteplayed=0;
  updatearray();
  displayarray();
  delay(20);
  for (int i=0; i<8 && noteplayed==0; i++)
  {
    if (boolarray[i]>0 /* && i != 5 */ ) 
    {
      tone(8, notefrequencies[i], 30);
      noteplayed=1;
    }
  }
  if (noteplayed==0) noTone(8); //no note played
  
  
}

/*
//Old loop function
void loop() {
  int distance1 = analogRead(A0);  
  if (distance1 > 200){
    tone(8, NOTE_C4, 20);
    }
  int distance2 = analogRead(A1);
  if (distance2 > 220){
    tone(8, NOTE_D4, 20);
    }
  int distance3 = analogRead(A2);
    if (distance3 > 200){
    tone(8, NOTE_E4, 20);
    }
  int distance4 = analogRead(A3);
    if (distance4 > 200){
    tone(8, NOTE_F4, 20);
    }
  int distance5 = analogRead(A4);
    if (distance5 > 200){
    tone(8, NOTE_G4, 20);
    }
  int distance6 = analogRead(A5);
    if (distance6 > 200){
    tone(8, NOTE_A5, 20);
    }
  int distance7 = analogRead(A6);
    if (distance7 > 200){
    tone(8, NOTE_B5, 20);
    }
}
*/
