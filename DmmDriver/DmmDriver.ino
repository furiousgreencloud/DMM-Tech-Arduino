#include "DmmDriver.h"

void setup() {
  Serial.begin(38600);
}

unsigned char statusByte = -1;
unsigned char configByte = -1;

void loop() {  
    const char Axis_Num = 0;
    unsigned char statusByte = -1;
    printf("DMM Test Motor\n\n");
    
    long result = ReadParamer(Read_Drive_Status, Axis_Num);
    bool FatalError = true;
    if (result != LONG_MIN ) {
        statusByte = (unsigned char)(result & 0x7f);
        FatalError = printStatusByte(statusByte);
    }
    
    if (FatalError || result == LONG_MIN) {
        delay(500);
        return;
        // STOP  MOTOR PISSED!
    }

    long delta = 5; //100;
    long center = 0;
    int delay_ms = 1000;

    long config = ReadParamer(Read_Drive_Config, Axis_Num);
    if (configByte >= 0 && config <= 0x7f) {
        configByte = (unsigned char)config;
    }
    
    
    
    // limited "Set" writes to firmware dont' use in loopDmmDriver_SerialPort_h
    //SetMainGain(Axis_Num, 60); //[15]// [14~20~40(loud)]relative to load, increase as load increases
    ReadParamer(Read_MainGain, Axis_Num);  ReadMainGain(Axis_Num); // Param is MotorID Main Gain stored in MainGain_Read variable
    // SetSpeedGain(Axis_Num, 127); // [127] higher : less dynamic movements
    ReadParamer(Read_SpeedGain, Axis_Num);
    ReadMotorPosition32(Axis_Num);
    
   // SetIntGain(Axis_Num, 1); // higher for rigid system, lower for loose system (outside disturbance)
                             // lag in feedback from encoder
    
    ReadParamer(Read_IntGain, Axis_Num);
    ReadParamer(Read_Pos_OnRange, Axis_Num);
    ReadParamer(Read_GearNumber, Axis_Num); // 500
    ReadParamer(Read_Drive_Config, Axis_Num);
    
#if false
    
    // reset Origin to Down/Reset Position
    ResetOrgin(Axis_Num);
    //return 0;
    
    // These are not remembered
    SetMaxSpeed(Axis_Num, 10); // 1
    SetMaxAccel(Axis_Num, 16); // 4

    
    //MoveMotorToAbsolutePosition32(Axis_Num,0);
    
    
    printf("\n");
    SetMaxSpeed(Axis_Num, 2); // 1
    SetMaxAccel(Axis_Num, 6); // 6
    
    MoveMotorConstantRotation(0,0);
    delay(50);
    return 0;
    for(;;) {
        if ((center % 3000) == 0) {
            Send_Package(General_Read, 0 , Is_AbsPos32);
            delta = delta * -1;
        }
        ReadPackage();
        delay(10);
     

//        SetMaxSpeed(Axis_Num, 1); // 1
//        SetMaxAccel(Axis_Num, 6); // 4
        
        center += delta;
        MoveMotorToAbsolutePosition32(Axis_Num, center);
    }
    
    MoveMotorToAbsolutePosition32(Axis_Num, center + delta);
//    ReadMotorTorqueCurrent(Axis_Num); //Motor torque current stored in MotorTorqueCurrent variable
    delay(delay_ms);

    MoveMotorToAbsolutePosition32(Axis_Num, center);
//    ReadMotorTorqueCurrent(Axis_Num); //Motor torque current stored in MotorTorqueCurrent variable
    delay(delay_ms);
 //   ReadMotorPosition32(Axis_Num); //Motor absolute position stored in Motor_Pos32 variable
    
    MoveMotorToAbsolutePosition32(Axis_Num, center + -1 * delta);
//    ReadMotorTorqueCurrent(Axis_Num); //Motor torque current stored in MotorTorqueCurrent variable
    delay(delay_ms);
 //   ReadMotorPosition32(Axis_Num); //Motor absolute position stored in Motor_Pos32 variable

    MoveMotorToAbsolutePosition32(Axis_Num, center);
        //   ReadMotorTorqueCurrent(Axis_Num); //Motor torque current stored in MotorTorqueCurrent variable
    delay(delay_ms);

        
    //}
    
#endif
    
    MotorDisengage(Axis_Num,configByte);
    delay(50);
}



