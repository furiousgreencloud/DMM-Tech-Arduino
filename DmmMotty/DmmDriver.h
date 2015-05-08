/*

Sample Code Notes:
(1) The sample code uses a ring buffer structure to input and output data
packet bytes. Two separate ring buffers are using in the code as char
InputBuffer[256] and char OutputBuffer[256].  Two position pointers are used in
each buffer structure to index the data inside the buffer structure. For
example, when a data packet is received from the servo drive, each byte
received is sequentially saved into the InputBuffer with the InBfTopPointer
incremented each time. This is done until the host hardware RS232 receiver
buffer is empty, meaning all packet bytes have been read and stored. Data is
processed as first-in-first-out (FIFO) queue and starts at the index of
InBfBtmPointer. InBfBtmPointer is incremented each time a byte is processed
until InBfBtmPointer=InBfTopPointer, meaning all packet bytes have been
processed.

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

#define bool unsigned short
#define true 1
#define false 0

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

ProtocolError_t Get_Function(void) ;
long Cal_SignValue(unsigned char One_Package[8]) ;
unsigned int Cal_UnsignedValue(unsigned char One_Package[8]) ;
void Make_CRC_Send(unsigned char Plength,unsigned char B[8]) ;
void ReadPackage() ;
ProtocolError_t Get_Function(void) ;
bool printStatusByte(unsigned char statusByte) ;
long Cal_SignValue(unsigned char One_Package[8] );
unsigned int Cal_UnsignedValue(unsigned char One_Package[8]) ;
void Send_Package(unsigned char func, char ID , long Displacement) ;
void Make_CRC_Send(unsigned char Plength,unsigned char B[8]) ;
void MoveMotorToAbsolutePosition32(char Axis_Num,long Pos32) ;
void MoveMotorConstantRotation(char Axis_Num,long r) ;
void ResetOrgin(char Axis_Num) ;
void SetMaxSpeed(char Axis_Num, int maxSpeed) ;
void SetMaxAccel(char Axis_Num, int maxAccel) ;
void SetMainGain(char Axis_Num, long gain) ;
void SetSpeedGain(char Axis_Num, long gain) ;
void SetIntGain(char Axis_Num, long gain) ;
void MotorDisengage(char Axis_Num, unsigned char curConfig) ;
void MotorEngage( char Axis_Num, unsigned char curConfig) ;
void ReadMainGain(char Axis_Num) ;
long ReadParamer(char queryParam, char Axis_Num) ;
void ReadMotorPosition32(char AxisID);

