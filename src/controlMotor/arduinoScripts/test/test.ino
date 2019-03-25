#include "motorClass.h"

// Robotzone motor
float gearRatio = 27;
float EncCntsPerRev = 48.0;

// create motor object
motorClass balMotor =  motorClass(6,5,8, gearRatio,EncCntsPerRev);

int controlType = 1; //0 = pos, 1 = vel

// limit switches
int limPin = 2; 
// by wiring through the normally open pin of the limit switches, we can combine them,
// early so that if either gets hit, it will send a 0. if neither are tripped, it will
// send a 4. if both are tripped it will still send a 0.

void setup () 
{
  // limit switches
  pinMode(limPin, INPUT_PULLUP);

  // calibrate position of weight
  // encPosOffset = weightCalibration();
  
  delay(1000);
  Serial.begin(9600);
}


void loop ()
{
  // control motor
  balMotor.setMotorVel(5);
  balMotor.vel_closedLoopController();
  
  // read limit switches hella fast
  int limVal = PIND;
  limVal = limVal & B00000100;
  
  // publish encoder position and limit switches to ros
  //balMotor.logValues();
  //Serial.println(balMotor.encodercount);
}



