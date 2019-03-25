#include "motorClass.h"

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

// Robotzone motor
float gearRatio = 27;
float EncCntsPerRev = 48.0;

// create motor object
motorClass balMotor =  motorClass(3,4,8,gearRatio,EncCntsPerRev);

int controlType = 1; //0 = pos, 1 = vel

// limit switches
int limPin = 2; 
// by wiring through the normally open pin of the limit switches, we can combine them,
// early so that if either gets hit, it will send a 0. if neither are tripped, it will
// send a 4. if both are tripped it will still send a 0.

void commandCallback(const std_msgs::Float32& setpoint)
{
    balMotor.setMotorVel(setpoint.data);
    //analogWrite(3,setpoint.data);
}

// ros stuff
ros::NodeHandle arduino2Motor;

// subscriber
ros::Subscriber<std_msgs::Float32> vel2ArduinoPub("vel2ArduinoPub", &commandCallback);

// publisher
std_msgs::Float32MultiArray ard2Control;
ros::Publisher ard2ControlPub("ard2ControlPub", &ard2Control);

// encoder position offset
int encPosOffset = -999;

void setup () 
{
  // limit switches
  pinMode(limPin, INPUT_PULLUP);
  
  // initialize ros
  arduino2Motor.initNode();
  
  // subscriber
  arduino2Motor.subscribe(vel2ArduinoPub);
  
  // publisher
  ard2Control.layout.dim[0].size = 2;
  ard2Control.data = (float *)malloc(sizeof(float)*2);
  ard2Control.data_length = 2;
  arduino2Motor.advertise(ard2ControlPub);

  // calibrate position of weight
  // encPosOffset = weightCalibration();
  
  delay(1000);

}


void loop ()
{
  // control motor
  balMotor.vel_closedLoopController();
  
  // read limit switches hella fast
  int limVal = PIND;
  limVal = limVal & B00000100;

  // test this 
  
  // publish encoder position and limit switches to ros
  ard2Control.data[0] = balMotor.encodercount;
  ard2Control.data[1] = limVal;

  ard2ControlPub.publish(&ard2Control);

  arduino2Motor.spinOnce();
}



