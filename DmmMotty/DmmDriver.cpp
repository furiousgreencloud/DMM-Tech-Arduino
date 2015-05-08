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

//#include <sysexits.h>
//#include <stdio.h>
#include <limits.h>
#include "Arduino.h"
#include "DmmDriver.h"

#define bool unsigned short
#define true 1
#define false 0


static char InputBuffer[8]; //Input buffer from RS232,
static signed int InBfTopPointer = 0,InBfBtmPointer = 0;//input buffer pointers
static unsigned char Read_Package_Buffer[8], Read_Num, Read_Package_Length;
long Drive_Read_Value = LONG_MIN;
unsigned char Drive_Read_Code = -1;
ProtocolError_t ProtocolError = Timeout_Error;

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
  if (Serial.available() < 0) { // no new data
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
  while(Serial.available () > 0 && ((InBfTopPointer+1) % sizeof(InputBuffer)) != InBfBtmPointer  ) {
      // while there is data and buffer not full
    InputBuffer[InBfTopPointer] = Serial.read(); //Load InputBuffer with received packets
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
  int i;
  char ID, ReceivedFunction_Code, CRC_Check;
  ID = Read_Package_Buffer[0]&0x7f;
  ReceivedFunction_Code = Read_Package_Buffer[1]&0x1f;
  CRC_Check = 0;
  for(i=0;i<Read_Package_Length-1;i++)
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
  switch(ReceivedFunction_Code) {
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

bool printStatusByte(unsigned char statusByte) {
    bool FatalError = false;
    printf("Motor Status\n");
    if (statusByte & 1) { // bit 0
        printf("\tMotor In position\n");
    } else {
        printf("\tMotor Out of Position\n");
    }
    
    if (statusByte & 2) { // bit 1
        printf("\tMotor Free/Disengaged\n");
    } else {
        printf("\tMotor Active/Enagaged\n");
    }
    
    if (statusByte & 28) { // bits 2,3,4
        printf("\tALARM: ");
        int alarmCode = (statusByte & 28) >> 2;
        switch (alarmCode) {
            case 1:
                printf("Lost Phase, |Pset - Pmotor|>8192(steps), 180(deg)\n");
                FatalError = true;
                break;
            case 2:
                printf("Over Current\n");
                FatalError = true;
                break;
            case 3:
                printf("Over Heat or Over Power\n");
                FatalError = true;
                break;
            case 4:
                printf("CRC Error Report, Command not Accepted\n");
                break;
            default:
                printf("Unkown Error\n");
        }
    } else {
        //printf("No Alarm");
    }
    
    if ((statusByte & 32) == 0) { // bit 5
        printf("\tWaiting for next S-curve,lieanr,circular motion\n");
    } else {
        printf("\tBUSY with current S-curve,lieanr,circular motion\n");
    }
    
    bool pin2JP3 = (statusByte & 64);  // bit 6
    printf("\tCNC Zero Position (PIN 2 of JP3): %s\n", (pin2JP3) ? "HIGH" : "LOW");
    return FatalError;
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
  int i;
  for(i=0;i<Plength-1;i++) {
    Serial.write(B[i]);
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

void MotorDisengage(char Axis_Num, unsigned char curConfig) {
    // Free Shaft
    Send_Package(Set_Drive_Config, Axis_Num, curConfig | Config_Bit_MOTOR_DRIVE);
}

void MotorEngage( char Axis_Num, unsigned char curConfig) {
    Send_Package(Set_Drive_Config, Axis_Num, curConfig & !Config_Bit_MOTOR_DRIVE);
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
        delay(20);
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


