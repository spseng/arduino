// ===============================
// MegaMotor6 Controller Program
// For Rhino Robot XR1 to XR4
// Written by Scott Savage
// Feb 2017 GNU General Public License (GPL)
// Ver 1.04 - Change log at end of file
// ===============================

#define MotorA 0
#define MotorB 1
#define MotorC 2
#define MotorD 3
#define MotorE 4
#define MotorF 5

//*************************************
// Define I/O lines
//*************************************
int Motor_IO_DIR[] = {11 ,A9 ,3  ,A1 ,17 ,51 }; // Motor Direction
int Motor_IO_PWM[] = {10 ,7  ,5  ,8  ,2  ,9  }; // Motor Speed (PWM)
int Motor_IO_BRK[] = {12 ,39 ,4  ,A2 ,16 ,50 }; // Motor Brake
int Motor_IO_CUR[] = {A5 ,A0 ,A6 ,A4 ,A7 ,A10}; // Motor Current Draw
int Motor_IO_THR[] = {14 ,A11,6  ,A3 ,15 ,52 }; // Motor Thermal Overload
int Motor_IO_LIM[] = {A8 ,26 ,28 ,30 ,40 ,38 }; // Motor Limit/Home Switch
int Motor_IO_QEA[] = {47 ,32 ,45 ,34 ,43 ,36 }; // Quadrature Encoder Channel A
int Motor_IO_QEB[] = {46 ,33 ,44 ,35 ,42 ,37 }; // Quadrature Encoder Channel B
int Expansion_IO[] = {A15,A14,A13,A12,53,49,48,41}; // Expansion Lines 
#define ESERT 18 // Serial Transmit
#define ESERR 19 // Serial Receive
#define EI2CD 20 // I2C Data
#define EI2CC 21 // I2C Clock
#define OPRLED 13 // Operate LED

//*************************************
// Motor Status / Motion Control Vars.
//*************************************
int DoPid = 0;
int Motion_Status[] = {0,0,0,0,0,0}; // Motion Status: 
int SyncMove_Status = 0;
#define AtTarget 0
#define BesideTarget 1 // Within 1 click.
#define CloseToTarget 2 // between 2 and 30 clicks.
#define OnApproachToTarget 3 // between 30 and 200 clicks.
#define OnWayToTarget 4 // More than 200 clicks away.
int Motor_Last_Dir[] = {0,0,0,0,0,0}; // Last Move Direction
int Motor_Position[6] __attribute__ ((section (".noinit"))); // Current Motor Positions
int Target_Position[] = {0,0,0,0,0,0}; // Target Motor Positions
int Motor_Speed[] = {0,0,0,0,0,0}; // Current Speed 
int Target_Speed[] = {0,0,0,0,0,0}; // Target Speed
int Motor_Current[] = {0,0,0,0,0,0}; // Motor Current Consumption.
int Current_PWM[] = {0,0,0,0,0,0}; // Current PWM
int PID_PError[] = {0,0,0,0,0,0}; // Proportional Error (Difference between Current and Target)
int PID_DValue[] = {0,0,0,0,0,0}; // 
int Limit_Prev[] = {0,0,0,0,0,0}; // Limit/Home switch Previous Value
int Limit_Bounce[] = {0,0,0,0,0,0}; // Limit/Home switch Previous Value
int Limit_For_On[] = {0,0,0,0,0,0}; // Limit/Home switch High Value
int Limit_For_Off[] = {0,0,0,0,0,0}; // Limit/Home switch High Value
int Limit_Rev_On[] = {0,0,0,0,0,0}; // Limit/Home switch Low Value
int Limit_Rev_Off[] = {0,0,0,0,0,0}; // Limit/Home switch Low Value
int QE_Prev[6] __attribute__ ((section (".noinit"))); ; // Quadrature Encoder Previous Read
int QE_Inc_States[] = {1, 3, 0, 2};
int QE_Dec_States[] = {2, 0, 3, 1};
int Forward_Logic[] = {0,0,0,0,0,0}; // Forward Logic - The value for the Direction IO Line when the motor needs to move forward.
int Reverse_Logic[] = {1,1,1,1,1,1}; // Reverse Logic - The value for the Direction IO Line when the motor needs to move Reverse.
int P1[] = {0,0,0,0,0,0}; // Testing Positions
int P2[] = {0,0,0,0,0,0}; // Testing Positions
int P3[] = {0,0,0,0,0,0}; // Testing Positions
int Start[] = {0,0,0,0,0,0};
int End[] = {0,0,0,0,0,0};
int Distance[] = {0,0,0,0,0,0};
float Ratio[] = {0,0,0,0,0,0};
#define MinSpeed 55
#define MaxError (255-MinSpeed)
#define MinError -(255-MinSpeed)
int Tracking = 0;
int Tracked[] = {0,0,0,0,0,0}; // Last Value send while tracking.
int DoSyncMove = 0;
int LeadMotor = 0;
int TravelSoFar = 0;
int TravelSoFarPrev = 0;
int tb = 0;
int tc = 0;
int td = 0;
int te = 0;
int tf = 0;

String InBuffer ="";
int Command_Motor = MotorE;
int savedFlag __attribute__ ((section (".noinit"))); 
//*************************************
// Initial Setup.
//*************************************
void setup(){
  
  pinMode(OPRLED, OUTPUT);
  pinMode(Expansion_IO[0], OUTPUT); // Tone

  InitMotorIO();
  
  //******************************************
  // TIMER SETUP - allows preceise timed 
  //  measurements of the Quadrature Encoder
  //******************************************
  cli();//stop interrupts
  //set timer1 interrupt at 1kHz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0;
  // set timer count for 2khz increments
  OCR1A = 1000;// = (16*10^6) / (2000*8) - 1
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for 8 prescaler
  TCCR1B |= (1 << CS11);   
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);  
  sei();//allow interrupts

  // Zero out saved variables on power cycle.
  if(savedFlag != 1234){  // 1234 is the magic number used to flag valid noinit data
    for (int m=0;m<=5;m++){
      Motor_Position[m]=0;
      QE_Prev[m]=0;
    }
    savedFlag = 1234; // Set savedFlag to the magic number.
  }

  Serial.begin(38400);
  Serial.println("Ready");  
  Serial.println("Type Help or ? for commands");
  ShowPositions();

}

//*************************************
// Main 
//*************************************
int lMotor = 0;
void loop(){
 // lMotor++; if (lMotor==6) lMotor=0;// Move to the next motor  
 // Motor_Current[lMotor] = analogRead(Motor_IO_CUR[lMotor]);

  if (Tracking>0) {
    TrackReport(lMotor);
  } 

  if (Serial.available()){
    int a= Serial.read();// read the incoming data as string
    if (a==13){
      Serial.println(InBuffer);
      if (InBuffer=="+"){
        int Position = Target_Position[Command_Motor]+10;
        MoveMotorTo(Command_Motor, Position);
      } else if (InBuffer=="-"){
        int Position = Target_Position[Command_Motor]-10;
        MoveMotorTo(Command_Motor, Position);
      } else if ((InBuffer=="G") || (InBuffer=="g")){
        TurnOnPID();        
      } else if ((InBuffer=="H") || (InBuffer=="h")){
        SetPositionToHome();
      } else if ((InBuffer=="I") || (InBuffer=="i")){
        InterrogateLimitSwitches();
      } else if ((InBuffer=="K") || (InBuffer=="k")){
        TestSeq2();
      } else if ((InBuffer=="O") || (InBuffer=="o")){
        OpenGripper();
      } else if ((InBuffer=="M") || (InBuffer=="m")){
        TestMotors();
      } else if ((InBuffer=="P") || (InBuffer=="p")){
        ShowPositions();
      } else if ((InBuffer=="Q") || (InBuffer=="q")){
        TestSeq1();
      } else if ((InBuffer=="S") || (InBuffer=="s")){
        TurnOffPID();
      } else if ((InBuffer=="T") || (InBuffer=="t")){
        DisplayStatus();
      } else if ((InBuffer=="U") || (InBuffer=="u")){
        StopTracking();
      } else if ((InBuffer=="V") || (InBuffer=="v")){
        CloseGripper();
      } else if ((InBuffer=="W") || (InBuffer=="w")){
        StartTracking();
      } else if ((InBuffer=="Z") || (InBuffer=="z")){
        ZeroPositions();
      } else if ((InBuffer=="Help") || (InBuffer=="help") || (InBuffer=="HELP") || (InBuffer == "?")){
        Serial.println("Commands:");
        Serial.println("A-F - Change motor position - [Motor] [Position]");
        Serial.println("a-f - Change motor angle - [Motor] [Angle]");
        Serial.println("G - Turn on motors");
        Serial.println("H - Set position to home");
        Serial.println("I - Interrogate limit switches");
        Serial.println("K - Test sequence 2");
        Serial.println("M - Test motors");
        Serial.println("O - Open gripper");
        Serial.println("P - Show positions");
        Serial.println("Q - Test sequence 1");
        Serial.println("S - Turn off motors");
        Serial.println("T - Display status");
        Serial.println("U - Stop tracking");
        Serial.println("V - Close gripper");
        Serial.println("W - Start tracking");
        Serial.println("Z - Zero positions");
      } else {
        char m = InBuffer.charAt(0);
        if ((m>=65) && (m<=70)) {
          Command_Motor = m - 65;
          InBuffer.setCharAt(0,32);
          int Position = InBuffer.toInt();
          MoveMotorTo(Command_Motor, Position);
        } else if ((m>=97) && (m<=102)) {
          Command_Motor = m - 97;
          InBuffer.setCharAt(0,32);
          float Angle = InBuffer.toFloat();
          MoveMotorToAngle(Command_Motor, Angle);
        }
      }
      InBuffer = "";
    } else {
      InBuffer = InBuffer + char(a);
    }
  }
}

//*****************************************
// Track the motors by sending the current 
//  position to a connected computer 
//*****************************************
void StartTracking() {
  if (InBuffer=="W"){
    Tracking=1; // Track Position 
    Serial.println("Tracking Positions");
  } else {
    Tracking=2; // Track Angles
    Serial.println("Tracking Angles");
  }
  // Set the last tracked position to some big number just  
  // to make the routine report the current position.
  for (int m=0;m<=5;m++){
    Tracked[m]=32000;
  }
}

void StopTracking(){
  Tracking=0; // Track none
  Serial.println("Tracking Off");
}

void TrackReport(int tMotor){
  if (Tracking>0){
    int Position = Motor_Position[tMotor];
    if (Tracked[tMotor]!=Position) {
      Serial.print("@");
      Serial.print(char(tMotor+65));
      if (Tracking==1) {
        Serial.println(Position);    
      } else if (Tracking==2) {
        Serial.println(PositionToAngle(tMotor,Position));    
      }    
      Tracked[tMotor]=Position;
    }
  }
}

//*************************************
// A test to see if the motor's current positions
//  are equal to the Target Positions.
// Which means that the motors are all at rest.
//*************************************
int AllAtTarget(){
  // Calculate and return a value that indicates that 
  // all motor have reached the target position.
  return ((Motion_Status[MotorA] == 0) && 
          (Motion_Status[MotorB] == 0) && 
          (Motion_Status[MotorC] == 0) && 
          (Motion_Status[MotorD] == 0) && 
          (Motion_Status[MotorE] == 0) && 
          (Motion_Status[MotorF] == 0));
}

//*******************************************************
// Set the I/O lines used by the LMD18200t Motor Driver
//*******************************************************
void InitMotorIO(){
  for (int m=0;m<=5;m++){
    pinMode(Motor_IO_BRK[m], OUTPUT);
    pinMode(Motor_IO_DIR[m], OUTPUT);
    pinMode(Motor_IO_PWM[m], OUTPUT);
    pinMode(Motor_IO_LIM[m], INPUT_PULLUP);
    pinMode(Motor_IO_THR[m], INPUT_PULLUP);
    pinMode(Motor_IO_QEA[m], INPUT_PULLUP);
    pinMode(Motor_IO_QEB[m], INPUT_PULLUP);
    digitalWrite(Motor_IO_BRK[m], HIGH); // Turn the Brakes on.
    digitalWrite(Motor_IO_DIR[m], HIGH); // Set the direction to Forward.
    digitalWrite(Motor_IO_PWM[m], LOW); // Set Speed to Zero.
  }  
}

//*******************************************************
// interrupt routine.
// Interrupts the main program at freq of 2kHz 
//*******************************************************
int iMotor = 0;
int LEDCounter = 0;
ISR(TIMER1_COMPA_vect) {
  //==========================================
  // Quadrature Encoders - read at rate of 2kHz 
  //==========================================
  for (int m=0;m<=5;m++){
    int Limit_Value = digitalRead(Motor_IO_LIM[m]);
    int QE_Value_A = digitalRead(Motor_IO_QEA[m]);
    int QE_Value_B = digitalRead(Motor_IO_QEB[m]) * 2;
    int QE_State = QE_Value_A + QE_Value_B;
    if (QE_State == QE_Inc_States[QE_Prev[m]]) {
      Motor_Last_Dir[m] = 1;
      Motor_Position[m]++;
      PID_DValue[m]=0;
    } else if (QE_State == QE_Dec_States[QE_Prev[m]]) {
      Motor_Last_Dir[m] = -1;
      Motor_Position[m]--;      
      PID_DValue[m]=0;
    } else {
      PID_DValue[m]++;
      if (PID_DValue[m]>10000) {
        PID_DValue[m]=10000;
      }
    }    
    QE_Prev[m] = QE_State;

    // See if Limit/Home switch changed
    if (Limit_Bounce[m] == Limit_Value) {
      if (Limit_Value != Limit_Prev[m]) {
        Limit_Prev[m] = Limit_Value;
        if (Motor_Last_Dir[m] == 1) {
          if (Limit_Value==0) {
            Limit_For_On[m] = Motor_Position[m]; 
          } else {
            Limit_For_Off[m] = Motor_Position[m]; 
          }        
        }      
        if (Motor_Last_Dir[m] == -1) {
          if (Limit_Value==0) {
            Limit_Rev_On[m] = Motor_Position[m];
          } else {
            Limit_Rev_Off[m] = Motor_Position[m];
          }                
        }      
      }    
    }
    Limit_Bounce[m] = Limit_Value;
  }

  //==========================================================
  // Blink the Operate LED
  //==========================================================
  LEDCounter++; // LEDCounter
  if (DoPid){
    // if the PID is on, then blink faster.
    if (!digitalRead(OPRLED)){
      LEDCounter+=4; // Count the Interrupts         
    }
    LEDCounter++; // Count the Interrupts   
  }
  if (LEDCounter>1000){
    LEDCounter-=1000;
    bool LEDState = !digitalRead(OPRLED);
    digitalWrite(OPRLED, LEDState);
    // Optional - Output the LED onto Expansion_IO 1.
    // This can be used to drive a speaker so 
    // that one can hear the PID status.
    digitalWrite(Expansion_IO[0], LEDState);
  }  

  //==========================================================
  // Calculate Motor status values.
  //==========================================================
  // Calculate only one motor per interupt - rate of 333Hz
  // This is done by stepping iMotor once per interupt. 
  // iMotor is then used to specifiy which motor to do 
  // calculations on.
  //==========================================================
  iMotor++; if (iMotor==6) iMotor=0;// Move to the next motor

  //==========================================================
  // Check for stall.
  // Note that the Gripper is excluded because it needs to 
  // apply tension on whatever it is gripping.
  //==========================================================
  if (iMotor>MotorA){    
    if (Motor_Current[iMotor] > 200) {
      if (Motor_Last_Dir[iMotor]==1) {
        Target_Position[iMotor] = Motor_Position[iMotor] - 50;
        Motor_Current[iMotor]=0;
      } else if (Motor_Last_Dir[iMotor]==-1) {
        Target_Position[iMotor] = Motor_Position[iMotor] + 50;
        Motor_Current[iMotor]=0;
      }
    }
  }

  //==========================================================
  // Calculate PID Proportional Error
  //==========================================================
  int PIDPError = Target_Position[iMotor]-Motor_Position[iMotor];
  PID_PError[iMotor] = PIDPError; // Save
    
  //==========================================================
  // Calc the Target Speed from the Proportional Error
  // The target speed is just the differnce between the 
  //  Current Position and the Target Position (with limits).
  // Results in a speed of +/- 255.
  //==========================================================
  if (PIDPError > MaxError) {
    Target_Speed[iMotor] = MaxError;
      // Set the Status that indicates that the Motor is more than 200 clicks from target.
      Motion_Status[iMotor] = OnWayToTarget;      
    
  } else if (PIDPError < MinError) {
    Target_Speed[iMotor] = MinError;
      // Set the Status that indicates that the Motor is more than 200 clicks from target.
      Motion_Status[iMotor] = OnWayToTarget;      
    
  } else if (PIDPError > 0) {
    Target_Speed[iMotor] = PID_PError[iMotor]+(PID_DValue[iMotor]/6);
    if (PIDPError < 2) {
      // Set the Status that indicates that the Motor is 1 click from target
      Motion_Status[iMotor] = BesideTarget;  
    } else if (PIDPError < 30) {
      // Set the Status that indicates that the Motor is 2-29 clicks from target
      Motion_Status[iMotor] = CloseToTarget;
    } else {
      // Set the Status that indicates that the Motor is 30-200 clicks from target
      Motion_Status[iMotor] = OnApproachToTarget;
    }
    
  } else if (PIDPError < 0) {
    Target_Speed[iMotor] =PID_PError[iMotor]-(PID_DValue[iMotor]/6),-255;
    if (PIDPError > -2) {
      // Set the Status that indicates that the Motor is 1 click from target
      Motion_Status[iMotor] = BesideTarget;  
    } else if (PIDPError > -30) {
      // Set the Status that indicates that the Motor is 2-29 clicks from target
      Motion_Status[iMotor] = CloseToTarget;
    } else {
      // Set the Status that indicates that the Motor is 30-200 clicks from target
      Motion_Status[iMotor] = OnApproachToTarget;
    }
    
  } else {
    Target_Speed[iMotor] = 0; 
    Motion_Status[iMotor] = AtTarget;  // Clear the flag that indicates that the Motor is in motion.

  }

  //==========================================================
  // PID (Currenty Just the P)
  //==========================================================
  if (DoPid){

    //============================================
    // Ramp Up/Down Current Speed to Target Speed.
    // Prevents the motors from jumping from dead 
    //  stop to full speed and vice-versa
    //============================================
    int CurrentSpeedChanged = 0;
    if (Target_Speed[iMotor]>Motor_Speed[iMotor]) {
      if (Motor_Speed[iMotor]<(255-MinSpeed)){
        Motor_Speed[iMotor]++; // if the target is higher, then inc up to the target.
        if (Target_Speed[iMotor]>Motor_Speed[iMotor]) {
          if (Motor_Speed[iMotor]<(255-MinSpeed)){
            Motor_Speed[iMotor]++; // if the target is higher, then inc up to the target a second time.
          }
        }
        CurrentSpeedChanged = 1;
      }
    } else if (Target_Speed[iMotor]<Motor_Speed[iMotor]) {
      if (Motor_Speed[iMotor]>-(255-MinSpeed)){
        Motor_Speed[iMotor]--; // if the target is lower, then inc down to the target.
        if (Target_Speed[iMotor]<Motor_Speed[iMotor]) {
          if (Motor_Speed[iMotor]>-(255-MinSpeed)){
            Motor_Speed[iMotor]--; // if the target is lower, then inc down to the target a second time.
          }
        }
        CurrentSpeedChanged = 1;
      }
    }

    if (CurrentSpeedChanged == 1){
      SetMotorPWM(iMotor);    
    }        
  }

  if (DoSyncMove>0) {    
    TravelSoFar =abs(Motor_Position[LeadMotor]-Start[LeadMotor]+1);
    if (TravelSoFar != TravelSoFarPrev) {
      TravelSoFarPrev = TravelSoFar;
      float TravelSoFarFloat = TravelSoFar;
      for (int m=1;m<=5;m++){
        if (m!=LeadMotor){
          float RP = TravelSoFarFloat*Ratio[m];
          int RI = int(RP);
          int TG = Start[m]+RI;
          Target_Position[m]=Start[m]+RI;
        }
      } 
      tb = abs(End[MotorB]-Motor_Position[MotorB]);
      tc = abs(End[MotorC]-Motor_Position[MotorC]);
      td = abs(End[MotorD]-Motor_Position[MotorD]);
      te = abs(End[MotorE]-Motor_Position[MotorE]);
      tf = abs(End[MotorF]-Motor_Position[MotorF]);            
      if ((tb==0) && (tc==0) && (td==0) && (te==0) && (tf==0)) {
        // Set the Status that indicates that the Motor is 1 click from target
        SyncMove_Status = AtTarget;
        DoSyncMove = 0;
      } else if ((tb<2) && (tc<2) && (td<2) && (te<2) && (tf<2)) {
        SyncMove_Status = BesideTarget;
      } else if ((tb<30) && (tc<30) && (td<30) && (te<30) && (tf<30)) {
        // Set the Status that indicates that the Motor is 2-29 clicks from target
        SyncMove_Status = CloseToTarget;
      } else if ((tb<200) && (tc<200) && (td<200) && (te<200) && (tf<200)) {
        // Set the Status that indicates that the Motor is 2-29 clicks from target
        SyncMove_Status = OnApproachToTarget;
      } else {
        // Set the Status that indicates that the Motor is 30-200 clicks from target
        SyncMove_Status = OnWayToTarget;
      }        
    }
  }
  
}

void SetMotorPWM(int m){
  //==========================================================
  // Calculate the PWM and Direction for the Speed
  // Converts the speed's +/- 255 value to PWM and Direction.
  //==========================================================  
  if (Motor_Speed[m] > 0) {      
    Current_PWM[m] = Motor_Speed[m]+MinSpeed;
    analogWrite(Motor_IO_PWM[m],Current_PWM[m]);  
    digitalWrite(Motor_IO_DIR[m],Forward_Logic[m]);          
  } else if (Motor_Speed[m] < 0) {      
    Current_PWM[m] = abs(Motor_Speed[m])+MinSpeed;
    analogWrite(Motor_IO_PWM[m],Current_PWM[m]);  
    digitalWrite(Motor_IO_DIR[m],Reverse_Logic[m]);        
  } else {
    Current_PWM[m] = 0;
    analogWrite(Motor_IO_PWM[m],0);          
  }  
}

void ZeroPositions() {
  for (int m=0;m<=5;m++){
    Motor_Position[m] = 0;
    Target_Position[m] = 0;
  }
  Serial.println("Current Positions set to Zero.");
}

void ShowPositions() {
  Serial.print("Current Positions: ");
  for (int m=0;m<=5;m++){
    Serial.print(char(m+65));
    Serial.print("=");
    Serial.print(Motor_Position[m]);
    if (m<5) Serial.print(",");
  }
  Serial.println(".");  
}

void SetPositionToHome() {
  for (int m=0;m<=5;m++){
    Target_Position[m] = 0;
  }  
  Serial.println("Setting Targets to Home Position.");
}

void TurnOffPID(){
  DoPid = 0;
  DoSyncMove = 0;
  for (int m=0;m<=5;m++){
    analogWrite(Motor_IO_PWM[m],0); // Set the speed to 0.
    digitalWrite(Motor_IO_BRK[m], LOW); // Turn the Brakes on.
  }
  Serial.println("Motors are now off.");
}

void TurnOnPID(){
  for (int m=0;m<=5;m++){
    digitalWrite(Motor_IO_BRK[m], LOW); // Turn the Brakes off.
  }
  DoPid = 1;
  Serial.println("Motors are now on.");
}

void OpenGripper(){
  Target_Position[MotorA] = -130;
  Serial.println("Opening Gripper.");
}

void CloseGripper(){
  Target_Position[MotorA] = -310;
  Serial.println("Closing Gripper.");
}

void DisplayStatus(){  
  Serial.println("Motor Status");
  Serial.print("Motors are ");
  if (DoPid==1) {
    Serial.println("on");  
  } else {
    Serial.println("off");  
  }
  for (int m=0;m<=5;m++){
  Serial.print(char(m+65));
  Serial.print(" Home=");  
  Serial.print(Limit_Prev[m]);
  Serial.print(": Sta=");
  Serial.print(Motion_Status[m]); // Report whether or not the Motor has reached the target location.
  Serial.print(" Pos=");
  Serial.print(Motor_Position[m]);
  Serial.print(" Tar=");  
  Serial.print(Target_Position[m]);
  Serial.print(" Err=");  
  Serial.print(PID_PError[m]);
  Serial.print(" Spd=");
  Serial.print(Motor_Speed[m]);
  //Serial.print(" TSpd=");
  //Serial.print(Target_Speed[m]);
  Serial.print(" PWM=");
  Serial.print(Current_PWM[m]);
  Serial.print(" Cur=");  
  Serial.print(Motor_Current[m]);
  Serial.print(" LmH=");
  Serial.print(Limit_Rev_Off[m]);
  Serial.print(",");
  Serial.print(Limit_For_On[m]);
  Serial.print(",");
  Serial.print(Limit_Rev_On[m]);
  Serial.print(",");
  Serial.print(Limit_For_Off[m]);
  Serial.print(",");
  Serial.print((Limit_For_Off[m] + Limit_Rev_On[m] + Limit_For_On[m] + Limit_Rev_Off[m]) / 4);
  Serial.println(" ");   
  } 
}

//************************************************************
// Cacluation to convert Rhino Robot angles to motor position
//************************************************************
int AngleToPosition(int Motor, float Angle) {
  float EncoderStepsPerDegree = 0;
  switch (Motor) {
    case MotorF: 
      EncoderStepsPerDegree = 17.5; //(4.4)(66.1/1)
      break;
    case MotorE:
    case MotorD:
    case MotorC:
      EncoderStepsPerDegree = 35; //(8.8)(66.1/1)
      break;
    case MotorB:
      EncoderStepsPerDegree = 12.5; //(5.51)(165.4/1)
      break;
    case MotorA:
      EncoderStepsPerDegree = 1;
  }
  return Angle * EncoderStepsPerDegree;
}

//************************************************************
// Cacluation to convert Rhino Robot motor positions to angles
//************************************************************
float PositionToAngle(int Motor, int Position) {
  float EncoderStepsPerDegree = 0;
  switch (Motor) {
    case MotorF: 
      EncoderStepsPerDegree = 17.5; //(4.4)(66.1/1)
      break;
    case MotorE:
    case MotorD:
    case MotorC:
      EncoderStepsPerDegree = 35; //(8.8)(66.1/1)
      break;
    case MotorB:
      EncoderStepsPerDegree = 12.5; //(5.51)(165.4/1)
      break;
    case MotorA:
      EncoderStepsPerDegree = 1;
  }
  return Position / EncoderStepsPerDegree;
}

//**********************************************
// Move a single Motor to a specified position. 
//**********************************************
void MoveMotorTo(int Motor, int Position) {
  Target_Position[Motor] = Position;
  Motion_Status[iMotor] = OnWayToTarget; //Set the flag that indicates that the motor has been put in motion.
  Serial.print(" ->Move Motor ");
  Serial.print(char(Motor+65));
  Serial.print(" to ");
  Serial.println(Position);  
}

//**********************************************
// Move a single Motor to a specified angle. 
//**********************************************
void MoveMotorToAngle(int Motor, float Angle) {
  int Position = AngleToPosition(Motor, Angle);
  MoveMotorTo(Motor, Position);
}

//***********************************************************
// Move Motors in synchronous mode by specifying the angles. 
// (Except the Gripper)
//***********************************************************
void SyncMoveAngle(float AngleB, float AngleC, float AngleD, float AngleE, float AngleF) {
  int PositionB = AngleToPosition(MotorB, AngleB);
  int PositionC = AngleToPosition(MotorC, AngleC);
  int PositionD = AngleToPosition(MotorD, AngleD);
  int PositionE = AngleToPosition(MotorE, AngleE);
  int PositionF = AngleToPosition(MotorF, AngleF);
  SyncMove(PositionB, PositionC, PositionD, PositionE, PositionF);
}

//**************************************************************
// Move Motors in synchronous mode by specifying the positions. 
// (Except the Gripper)
//**************************************************************
void SyncMove(int PositionB, int PositionC, int PositionD, int PositionE, int PositionF) {
  DoSyncMove = 0;  
  SyncMove_Status = OnWayToTarget;
  // Store the target positions.
  End[MotorB]=PositionB;
  End[MotorC]=PositionC;
  End[MotorD]=PositionD;
  End[MotorE]=PositionE;
  End[MotorF]=PositionF;
  float MaxDistance = 0;
  // Caculate the travel distance for each motor.
  for (int m=1;m<=5;m++){
    Start[m]=Target_Position[m];
    Distance[m]=End[m]-Start[m];
    // Keep track of the furthest travel distance.
    MaxDistance = max(MaxDistance,abs(Distance[m]));
  }
  // Using the motor with the furthest travel distance,
  // caculate the ratios of travel distance between all motors.
  for (int m=1;m<=5;m++){
    Ratio[m] = Distance[m]/MaxDistance;
    if (abs(Distance[m])==MaxDistance) {
      LeadMotor = m;
    }
    Serial.print(char(m+65));
    Serial.print(": From:");
    Serial.print(Start[m]);
    Serial.print(" To:");
    Serial.print(End[m]);
    Serial.print(" Distance:");
    Serial.print(Distance[m]);
    Serial.print(" Speed Ratio:");  
    Serial.println(Ratio[m] * 100);
  }  
  Target_Position[LeadMotor] = End[LeadMotor];  
  SyncMove_Status = OnWayToTarget;
  DoSyncMove = 1;
  Serial.println("Start Sync Move");
}

void InterrogateLimitSwitches(){
  Serial.println("Interrogate Limit Switches");  
  int CurrentMotorState = DoPid;
  TurnOnPID();  
  InterrogateLimitSwitch(MotorF, 450, -450);
  MoveMotorTo(MotorD,-300);
  InterrogateLimitSwitch(MotorE, 280, -280);
  MoveMotorTo(MotorD,-0);
  MoveMotorTo(MotorE,-300);
  delay(500);
  InterrogateLimitSwitch(MotorD, 320, -320);
  MoveMotorTo(MotorE,0);
  InterrogateLimitSwitch(MotorC, 300, -300);
  InterrogateLimitSwitch(MotorB, 380, -380);
  Serial.println("Done Interrogating Limit Switches");  
  if (CurrentMotorState == 0) {
    TurnOffPID();
  }  
}

void InterrogateLimitSwitch(int m, int f, int r) {
  if (Limit_Prev[m]==0){
    Serial.print(" Centering Motor ");
    Serial.println(char(m+65));
    MoveMotorTo(m,r-30);
    do {TrackReport(m);} while (Motor_Position[m] > r);
    MoveMotorTo(m,f+30);
    do {TrackReport(m);} while (Motor_Position[m] < f);
    MoveMotorTo(m,r-30);
    do {TrackReport(m);} while (Motor_Position[m] > r);
    int NewHome = ((Limit_For_Off[m] + Limit_Rev_On[m] + Limit_For_On[m] + Limit_Rev_Off[m]) / 4);
    MoveMotorTo(m,NewHome);
    do {TrackReport(m);} while (Motor_Position[m] != NewHome);
    Motor_Position[m] = 0;
    Target_Position[m] = 0;
    Serial.print(" Motor ");
    Serial.print(char(m+65));
    Serial.println(" Centered");
  } else {
    Serial.print(" Motor ");
    Serial.print(char(m+65));
    Serial.println(" Not Centered");
  }
}

void TestMotors() {
  Serial.println("Test Motors");    
  TurnOffPID();
  delay(250);

  ShowPositions();

  // Set all motor power lines to High-Z state by 
  // Setting speed to 0 and turning off brakes.  
  Serial.println("Setting Drive Power for all Motors to High-Z.");
  for (int m=0;m<=5;m++){    
    digitalWrite(Motor_IO_BRK[m], LOW); // Turn the Brakes off.
    Motor_Speed[m] = 0;
    SetMotorPWM(m);
  }  
  
  for (int m=0;m<=5;m++){    
    TestMotor(m);
    delay(1);
  }

  Serial.print("Forward encoder count");    
  for (int m=0;m<=5;m++){
    Serial.print(":");
    Serial.print(P3[m]-P2[m]);
  } 
  Serial.println("."); 
  
  Serial.print("Reverse encoder count");
  for (int m=0;m<=5;m++){
    Serial.print(":");
    Serial.print(P2[m]-P1[m]);
  } 
  Serial.println("."); 
  ShowPositions();
  Serial.println("Done Testing Motors");  
}

void TestMotor(int m) {
  int TestSpeed = 255-MinSpeed;
  int SpeedDelay = 50;
  Serial.print("Moving Motor ");
  Serial.println(char(m+65));
  Serial.print("   backward");
  
  P1[m] = Motor_Position[m]; // Get Current Position.
  
  Motor_Speed[m]=-TestSpeed; // Set Motor Speed.
  SetMotorPWM(m);            // Set Motor PWM.
  Serial.print(" On");
  delay(SpeedDelay);         // Short Delay to allow the motor to move.  
    
  Motor_Speed[m]=0;          // Turn off motor.
  SetMotorPWM(m);            // Set Motor PWM.
  Serial.println(" Off");
  delay(SpeedDelay);         // Short Delay to allow the motor to stop.  

  P2[m] = Motor_Position[m]; // Get Current Position.

  Serial.print("   forward ");
  //Serial.print("Moving Motor ");
  //Serial.print(char(m+65));
  //Serial.print(" forward ");

  Motor_Speed[m]=TestSpeed;  // Set Motor Speed.
  SetMotorPWM(m);            // Set Motor PWM.
  Serial.print(" On");
  delay(SpeedDelay);         // Short Delay to allow the motor to move.  
    
  Motor_Speed[m]=0;          // Turn off motor.
  SetMotorPWM(m);            // Set Motor PWM.
  Serial.println(" Off");
  delay(SpeedDelay);         // Short Delay to allow the motor to stop.  

  P3[m] = Motor_Position[m]; // Get Current Position.

}

// ****************************************************
// ****************************************************
//
//                  Move Sequences
//
// ****************************************************
// ****************************************************

void TestSeq1() {
  for (int m=0;m<=50;m++){
    Serial.println(m);
    TestSeq1b();
  }
}

void TestSeq1b() {
Serial.println("start");
  TurnOffPID();
  OpenGripper();
  MoveMotorToAngle(MotorB,90);
  MoveMotorToAngle(MotorE,-130);
  MoveMotorToAngle(MotorF,45);
  TurnOnPID();
  do {delay(50);} while (Motion_Status[MotorE] > OnApproachToTarget); 
  MoveMotorToAngle(MotorD,17.1);
  do {delay(50);} while (Motion_Status[MotorD] > OnApproachToTarget); 
  CloseGripper();
  delay(1000);
  MoveMotorToAngle(MotorD,0);
  MoveMotorToAngle(MotorE,-80);
  do {delay(50);} while (Motion_Status[MotorD] > OnApproachToTarget); 
  MoveMotorToAngle(MotorF,-45);
  do {delay(50);} while (Motion_Status[MotorE] > OnApproachToTarget); 
  OpenGripper();
  delay(1000);
  SetPositionToHome();
  do {delay(50);} while (Motion_Status[MotorE] > BesideTarget); 
  TurnOffPID(); 
  Serial.println("stop");
}

void TestSeq2() {
  Serial.println("start Seq 2");
  TurnOnPID();    
  SyncMoveAngle(90,0,0,-130,45);
  do {delay(50);} while (SyncMove_Status > BesideTarget);
  TurnOffPID();  
  Serial.println("Done");
}

// Change log
// 1.02 3/19/2017 line 352: Excluded Motor A from the stall detect routine because it is the gripper and closing on an item is technically a stall.
// 1.03 3/31/2017 line 792: Added TestMotors routine to test motors that are in an electrically unknown state.
// 1.04 4/01/2017 line 792: Changed TestMotors routine so that all motor Drives are High-Z except the ones under test.
