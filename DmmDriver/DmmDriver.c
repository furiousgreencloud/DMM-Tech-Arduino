/*

Sample Code Notes:
(1) The sample code uses a ring buffer structure to input and output data packet bytes. Two separate ring buffers are
using in the code as char InputBuffer[256] and char OutputBuffer[256].
Two position pointers are used in each buffer structure to index the data inside the buffer structure. For example, when
a data packet is received from the servo drive, each byte received is sequentially saved into the InputBuffer with the
InBfTopPointer incremented each time. This is done until the host hardware RS232 receiver buffer is empty, meaning
all packet bytes have been read and stored. Data is processed as first-in-first-out (FIFO) queue and starts at the index
of InBfBtmPointer. InBfBtmPointer is incremented each time a byte is processed until InBfBtmPointer=InBfTopPointer,
meaning all packet bytes have been processed.

Communication Format
	Baud Rate 38400
	Start/Stop Bit	1
	Odd/Even Verify Bit	No
	Data	8-bit
	Full Duplex
	Asynchronous
	Voltage	TTL/CMOS/5V

*/

#include <sysexits.h>
#include <stdio.h>
#include <limits.h>

#include "SerialPort.h"

#define Go_Absolute_Pos 0x01
#define Turn_ConstSpeed 0x0a
#define Set_Origin 0x00
#define Set_HighSpeed 0x14
#define Set_HighAccel 0x15
#define Set_MainGain  0x10
#define Set_SpeedGain 0x11
#define Set_IntGain  0x12


#define Set_Drive_Config 0x07
#define Config_Bit_MOTOR_DRIVE 0x10 // HIGH: Motor Drive Enabled, LOW: Motor Drive Free

#define General_Read 0x0e

#define Is_AbsPos32 0x1b
#define Is_TrqCurrent 0x1e
#define Is_MainGain 0x10
#define Is_SpeedGain 0x11
#define Is_IntGain 0x12
#define Is_Status 0x19
#define Is_Config 0x1a
#define Is_PosOn_Range 0x17
#define Is_GearNumber 0x18
#define Is_TrqCons 0x13
#define Is_HighSpeed 0x14
#define Is_HighAccel 0x15
#define Is_Drive_ID 0x16

#define Read_MainGain 0x18
#define Read_SpeedGain 0x19
#define Read_IntGain 0x1a
#define Read_Drive_Config 0x08
#define Read_Drive_Status 0x09
#define Read_Pos_OnRange 0x1e
#define Read_GearNumber 0x1f
#define Read_Drive_ID 0x06

#ifndef MIN
    #define MIN(a,b) ((a<b) ? a : b)
#endif

#ifndef MAX
    #define MAX(a,b) ((a>b) ? a : b)
#endif

typedef enum {In_Progress = 0, Complete_Success,  CRC_Error, Timeout_Error } ProtocolError_t;

static char InputBuffer[8]; //Input buffer from RS232,
static signed int InBfTopPointer = 0,InBfBtmPointer = 0;//input buffer pointers
static unsigned char Read_Package_Buffer[8], Read_Num, Read_Package_Length;
long Drive_Read_Value = LONG_MIN;
unsigned char Drive_Read_Code = -1;
ProtocolError_t ProtocolError = Timeout_Error;
unsigned char configByte = 0;

// Forwards
ProtocolError_t Get_Function(void);
long Cal_SignValue(unsigned char One_Package[8]);
unsigned int Cal_UnsignedValue(unsigned char One_Package[8]);
void Make_CRC_Send(unsigned char Plength,unsigned char B[8]);

const char * ParameterName(char isCode) {
    switch(isCode) {
        case Is_MainGain : return "Main Gain";
        case Is_SpeedGain : return "Speed Gain";
        case Is_IntGain : return "Intergration Gain";
        case Is_Status : return "Status Byte";
        case Is_Config : return "Config Byte";
        case Is_PosOn_Range : return "Position On Range";
        case Is_GearNumber : return "Gear Number";
        case Is_AbsPos32 : return "Absolute Position";
        case Is_TrqCons : return "Torque Contant";
        case Is_HighSpeed : return "Max Speed";
        case Is_HighAccel : return "Max Acceleration";
        case Is_Drive_ID : return "Drive ID";
        default: return "Unknown Parameter";
    }
}


void ReadPackage() {
  unsigned char c,cif;
  if (SerialAvailable() == 0) { // no new data
      return;
      /*
       delay(50);
      printf("... \n");
      if (SerialAvailable() == 0) {
          ProtocolError = Timeout_Error;
          printf("Timeout\n");
          return;
      }
      */
  }
  while(SerialAvailable() > 0 && ((InBfTopPointer+1) % sizeof(InputBuffer)) != InBfBtmPointer  ) {
      // while there is data and buffer not full
    InputBuffer[InBfTopPointer] = SerialRead(); //Load InputBuffer with received packets
    InBfTopPointer++;  InBfTopPointer %=sizeof(InputBuffer);
  }
  while(InBfBtmPointer != InBfTopPointer)  { // while not empty
    c = InputBuffer[InBfBtmPointer];
    InBfBtmPointer++; InBfBtmPointer %=sizeof(InputBuffer);;
    cif = c&0x80; // Start or "End" Frame Char
    if(cif==0) {
      Read_Num = 0;
      Read_Package_Length = 0;
    }
    if(cif==0||Read_Num>0) {
      Read_Package_Buffer[Read_Num] = c;
      Read_Num++;
      if(Read_Num==2)
      {
        cif = c>>5;
        cif = cif&0x03;
        Read_Package_Length = 4 + cif;
        c = 0;
      }
      if(Read_Num==Read_Package_Length)
      {
        ProtocolError = Get_Function();
        Read_Num = 0;
        Read_Package_Length = 0;
        if (ProtocolError != In_Progress) {
//            InBfBtmPointer = InBfTopPointer = 0;
            return;
        }
      }
    }
  }
}

ProtocolError_t Get_Function(void)
{
  char ID, ReceivedFunction_Code, CRC_Check;
  ID = Read_Package_Buffer[0]&0x7f;
  ReceivedFunction_Code = Read_Package_Buffer[1]&0x1f;
  CRC_Check = 0;
  for(int i=0;i<Read_Package_Length-1;i++)
  {
    CRC_Check += Read_Package_Buffer[i];
  }
  CRC_Check ^= Read_Package_Buffer[Read_Package_Length-1];
  CRC_Check &= 0x7f;
  
  if(CRC_Check!= 0){
    //MessageBox(?There is CRC error!?) - Customer code to indicate CRC error
      printf("CRC Error\n");
      return CRC_Error;
  }
  Drive_Read_Code = (unsigned char)ReceivedFunction_Code;
  switch(ReceivedFunction_Code){
        case Is_AbsPos32:
        case Is_TrqCurrent:
        case Is_GearNumber:
        case Is_Config :
        case Is_Status :
            Drive_Read_Value = Cal_SignValue(Read_Package_Buffer);
            break;
        case Is_MainGain:
        case Is_SpeedGain:
        case Is_IntGain:
        case Is_TrqCons:
        case Is_HighSpeed:
        case Is_HighAccel:
        case Is_Drive_ID:
        case Is_PosOn_Range:
            Drive_Read_Value = Cal_UnsignedValue(Read_Package_Buffer);
            break;
        default:
            Drive_Read_Value = Cal_SignValue(Read_Package_Buffer);
  }
  printf("%s: %ld\n",ParameterName(Drive_Read_Code), Drive_Read_Value);
  return Complete_Success;
}

void printStatusByte(unsigned char statusByte) {
    if (statusByte & 1) { // bit 0
        printf("Motor in Position\n");
    } else {
        printf("Motor out of Position\n");
    }
    
    if (statusByte & 2) { // bit 1
        printf("Motor Active\n");
    } else {
        printf("Motor Free\n");
    }
    
    if (statusByte & 28) { // bits 2,3,4
        printf("Alarm: ");
        int alarmCode = (statusByte & 28) >> 2;
        switch (alarmCode) {
            case 1: printf("Lost Phase, |Pset - Pmotor|>8192(steps), 180(deg)\n"); break;
            case 2: printf("Over Current\n"); break;
            case 3: printf("Over Head or Over Power\n");
            case 4: printf("CRC Error Report, Command no Accepted\n");
            default:
                printf("Unkown Error\n");
        }
    } else {
        printf("No Alarm");
    }
    
    if ((statusByte & 32) == 0) { // bit 5
        printf("S-curve,lieanr,circular motion waiting for next motion\n");
    } else {
        printf("S-curve,lieanr,circular motion busy with current motion\n");
    }
    
    Boolean pin2JP3 = (statusByte & 64);  // bit 6
    printf("PIN 2 of JP3 (CNC Zero Position) is %s\n", (pin2JP3) ? "HIGH" : "LOW");
}

/*Get data with sign - long*/
long Cal_SignValue(unsigned char One_Package[8])
{
  char Package_Length,OneChar,i;
  long Lcmd;
  OneChar = One_Package[1];
  OneChar = OneChar>>5;
  OneChar = OneChar&0x03;
  Package_Length = 4 + OneChar;
  OneChar = One_Package[2]; /*First byte 0x7f, bit 6 reprents sign */
  OneChar = OneChar << 1;
  Lcmd = (long)OneChar; /* Sign extended to 32bits */
  Lcmd = Lcmd >> 1;
  for(i=3;i<Package_Length-1;i++)
  {
    OneChar = One_Package[i];
    OneChar &= 0x7f;
    Lcmd = Lcmd<<7;
    Lcmd += OneChar;
  }
  return(Lcmd); /* Lcmd : -2^27 ~ 2^27 - 1 */
}

/*Get data with out  - unsigned */
unsigned int Cal_UnsignedValue(unsigned char One_Package[8])
{
    char Package_Length,OneChar,i;
    unsigned int ret;
    OneChar = One_Package[1];
    OneChar = OneChar>>5;
    OneChar = OneChar&0x03;
    Package_Length = 4 + OneChar;
    OneChar = One_Package[2] & 0x7f; // only 7 bits are of data bits
    ret = (unsigned int)OneChar;
    for(i=3;i<Package_Length-1;i++)
    {
        OneChar = One_Package[i];
        OneChar &= 0x7f;
        ret = ret<<7;
        ret += OneChar;
    }
    return(ret); /* ret : 0 ~ 127 */
}


// ***************** Every Robot Instruction ******************
// Send a package with a function by Global_Func
// Displacement: -2^27 ~ 2^27 - 1
// Note: in the description of RS232 communication protocol above (Section 7), the last byte of packet is
// always B0, but in the code of below, the first byte is always B0.
//

void Send_Package(unsigned char func, char ID , long Displacement)
{
    
  unsigned char B[8],Package_Length,Function_Code;
  long TempLong;
  B[1] = B[2] = B[3] = B[4] = B[5] = (unsigned char)0x80;
  B[0] = ID&0x7f;
  Function_Code = func & 0x1f;
  TempLong = Displacement & 0x0fffffff; //Max 28bits
  B[5] += (unsigned char)TempLong&0x0000007f;
  TempLong = TempLong>>7;
  B[4] += (unsigned char)TempLong&0x0000007f;
  TempLong = TempLong>>7;
  B[3] += (unsigned char)TempLong&0x0000007f;
  TempLong = TempLong>>7;
  B[2] += (unsigned char)TempLong&0x0000007f;
  Package_Length = 7;
  TempLong = Displacement;
  TempLong = TempLong >> 20;
  if(( TempLong == 0x00000000) || ( TempLong == 0xffffffff))
  {//Three byte data
    B[2] = B[3];
    B[3] = B[4];
    B[4] = B[5];
    Package_Length = 6;
  }
  TempLong = Displacement;
  TempLong = TempLong >> 13;
  if(( TempLong == 0x00000000) || ( TempLong == 0xffffffff))
  {//Two byte data
    B[2] = B[3];
    B[3] = B[4];
    Package_Length = 5;
  }
  TempLong = Displacement;
  TempLong = TempLong >> 6;
  if(( TempLong == 0x00000000) || ( TempLong == 0xffffffff))
  {//One byte data
    B[2] = B[3];
    Package_Length = 4;
  }
  B[1] += (Package_Length-4)*32 + Function_Code;
  Make_CRC_Send(Package_Length,B);
}


void Make_CRC_Send(unsigned char Plength,unsigned char B[8]) {
  unsigned char Error_Check = 0;
  for(int i=0;i<Plength-1;i++) {
    SerialWrite(B[i]);
    Error_Check += B[i];
  }
  Error_Check = Error_Check|0x80;
  SerialWrite(Error_Check);
}


/*
void ReadMotorTorqueCurrent(char AxisID)  {
    
  // Below are the codes for reading the motor torque current
  //Read motor torque current
  Send_Package(General_Read, AxisID , Is_TrqCurrent);
  //Function code is General_Read, but one byte data is : Is_TrqCurrent
  //Then the drive will return a packet, Function code is Is_TrqCurrent
  //and the data is 16bits Motor torque current.
  while(ProtocolError == In_Progress) {
    ReadPackage();
  }
}

 */


void MoveMotorToAbsolutePosition32(char Axis_Num,long Pos32) {
//  printf("Going To: %ld\n",Pos32);
  Send_Package(Go_Absolute_Pos, Axis_Num, Pos32);
}

void MoveMotorConstantRotation(char Axis_Num,long r) {
// TODO set Limits for 3 byte value of r
    Send_Package(Turn_ConstSpeed, Axis_Num, r);
}


void ResetOrgin(char Axis_Num) {
  Send_Package(Set_Origin, Axis_Num, 0); // 0: Dummy Data
}

void SetMaxSpeed(char Axis_Num, int maxSpeed) {
  long m = MAX(1,MIN(127,maxSpeed));
  Send_Package(Set_HighSpeed, Axis_Num, m);
}

void SetMaxAccel(char Axis_Num, int maxAccel) {
    long m = MAX( 1, MIN( 127, maxAccel));
    Send_Package(Set_HighAccel, Axis_Num, m);
}

void SetMainGain(char Axis_Num, long gain) {
    long l = MAX( 1, MIN( 127, gain));
    Send_Package(Set_MainGain, Axis_Num, l);
}

void SetSpeedGain(char Axis_Num, long gain) {
    long l = MAX( 1, MIN( 127, gain));
    Send_Package(Set_SpeedGain, Axis_Num, l);
}

void SetIntGain(char Axis_Num, long gain) {
    long l = MAX(1, MIN(127, gain));
    Send_Package(Set_IntGain, Axis_Num, l);
}

void ReadMainGain(char Axis_Num) {
  Send_Package(Read_MainGain, Axis_Num, Is_MainGain);
  ProtocolError = In_Progress;
  while(ProtocolError == In_Progress) {
      ReadPackage();
  }
}

long ReadParamer(char queryParam, char Axis_Num) {
    ProtocolError = In_Progress;
    Drive_Read_Value = -1;
    Drive_Read_Value = -1;
    Send_Package(queryParam, Axis_Num, 0 ); // 0 is a dummy Data Value
    while(ProtocolError == In_Progress) {
        ReadPackage();
    }
//    printf("%s: %ld\n",ParameterName(Drive_Read_Code), Drive_Read_Value);
    if (ProtocolError == Complete_Success) {
        return Drive_Read_Value;
    } else {
        return LONG_MIN;
    }
}

void ReadMotorPosition32(char AxisID)
{ // Below are the codes for reading the motor shaft 32bits absolute position
    //Read motor 32bits position
    ProtocolError = In_Progress;
    Send_Package(General_Read, AxisID , Is_AbsPos32);
    // Function code is General_Read, but one byte data is : Is_AbsPos32
    // Then the drive will return a packet, Function code is Is_AbsPos32
    // and the data is 28bits motor position32.
    while(ProtocolError == In_Progress) {
        ReadPackage();
    }
}



int testMotor(void)
{
    printf("DMM Test Motor\n\n");
    
    const char Axis_Num = 0;
     long delta = 5; //100;
     long center = 0;
    const int delay_ms = 1000;

    long config = ReadParamer(Read_Drive_Config, Axis_Num);
    if (configByte >= 0 && config <= 0x7f) {
        configByte = (unsigned char)config;
    }
    

    // limited "Set" writes to firmware dont' use in loopDmmDriver_SerialPort_h
    SetMainGain(Axis_Num, 60); //[15]// [14~20~40(loud)]relative to load, increase as load increases
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
    // reset Origin to Down/Reset Position
    ResetOrgin(Axis_Num);
    //return 0;
    
    // These are not remembered
    SetMaxSpeed(Axis_Num, 10); // 1
    SetMaxAccel(Axis_Num, 16); // 4
    
    
    
    //MoveMotorToAbsolutePosition32(Axis_Num,0);
    
    ReadParamer(Read_Drive_Status, Axis_Num);
    printStatusByte(Drive_Read_Value);
    if (Drive_Read_Value != 0) {
//        return (int)Drive_Read_Value;
    }
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
    return 0;
}


int main() {
    int ret = 0;
    if (openSerial("/dev/cu.usbserial") == EX_OK) {
         ret = testMotor();
        closeSerial();
    }
    return ret;
}