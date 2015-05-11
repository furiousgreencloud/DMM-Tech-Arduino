#include "DmmDriver.h"

unsigned char statusByte = -1;
unsigned char configByte = -1;
const char Axis_Num = 0;


void setup() {
  Serial.begin(38600);
  delay(1000);
  SetMainGain(Axis_Num, 1);
  SetIntGain(Axis_Num, 1); // higher for rigid system, lower for loose system (outside disturbance)
                             // lag in feedback from encoder
                             
  SetSpeedGain(Axis_Num, 127); // [127] higher : less dynamic movements
  SetMaxSpeed(Axis_Num, 2); // 1
  SetMaxAccel(Axis_Num, 4); // 4
  MotorEngage(Axis_Num,0);
}


void loop() {  
   // these are fogorrent on power reset to we just send them 
    SetMaxSpeed(Axis_Num, 2); // 1
    SetMaxAccel(Axis_Num, 4); // 4
  
 #if false // Rotation Test
    MoveMotorConstantRotation(Axis_Num,10);
    delay(2000);
    MoveMotorConstantRotation(Axis_Num,-10);
    delay(2000);
 #endif
    
#if true // Abs Pos Test
    MoveMotorToAbsolutePosition32(Axis_Num, -1000);
    delay(4000);
    MoveMotorToAbsolutePosition32(Axis_Num, 1000);    
    delay(4000);
#endif
    
  
}



