#include <SFE_LSM9DS0.h>



//robot randomly moves around and avoids obstacles
//uses two motors (A0,A1)
//and two distance sensors (A2,A7)
//motors are controlled via the servo library and treated as continuous servos 
// 0-full speed 1 way 90-neutral/not moving 180-full speed the opposite direction
// the motors are facing opposite directions, so speeds must be inverse

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Simple_AHRS.h>

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B

// Create sensor instances.
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);


LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

Servo motorR;
Servo motorL;

void setup() {
  Serial.begin(115200);
  motorR.attach(A0);
  motorL.attach(A1);
  accel.begin();
  mag.begin();
}

void loop() {

  
  int motorset = 90;
  int right = 90-motorset;
  int left = 90+motorset;
  
  while(distance()!=1 &&gyrotest()!=1){       //while the distance sensor doesn't see anything  
    motorR.write(right);        //make both motors go forwards at the same speed
    motorL.write(left);
  }
                                  /// when something is in front of the distance sensor
  stopmotors();                 //stop going forward
  delay(500);
  
  motorR.write(left);           //go backwards 
  motorL.write(right);
  delay(random(500,1500));
  stopmotors();                         ////stop going backwards
  
  turn();                                    ////turn than stop
}                                       ///repeat loop







int gyrotest(){
  int i;
  int lastacceleration= dof.calcAccel(dof.az);
  
  for (i = 0; i < 500; i = i + 1) {
          if (dof.calcAccel(dof.az)<10+lastacceleration&&dof.calcAccel(dof.az)>lastacceleration-10){
            i = i+2;
            return 1;
          }
          else{
            return 0;
          }
      }
}





void turn(){
  motorR.write(random(0,180));      /////robot turns random direction for less than stops moving
  motorL.write(random(0,180));
  delay(random(500,2000));
  stopmotors();
}




int distance(){            //////checks the distance sensors, returns 0 if there is nothing close to either sensor
      int back=0;          ////// and returns 1 if there is something close to either sensor
      int i;
      int distance1 = analogRead(A2);
      int distance2 = analogRead(A7);
      int distancesensors[3]={distance1,distance2};
      for (i = 0; i < 2; i = i + 1) {
          if (distancesensors[i]>450){
            i = i+2;
            return 1;
          }
          else{
            return 0;
          }
      }
  }




void stopmotors(){      ////returns both motors to neutral 
  motorR.write(90);
  motorL.write(90);
}

