/*
  mx28serial1.h

  ArbotiX library for AX/RX control.
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

	8-oct-2013

	Modified by David Sanz Kirbis to work with serial1

	using modified bioloid ax12 library to setup your MX-28 servos
	with a Leonardo board and Serial1 through an 74ls241

	for the hardware setup see:
	http://savageelectronics.blogspot.com.es/2011/01/arduino-y-dynamixel-ax-12.html

	for original library see:
	http://forums.trossenrobotics.com/showthread.php?5812-Help-with-controlling-an-Dynamixel-MX-28T-with-an-Arduino-Uno

*/

#include <Arduino.h>

#ifndef mx28serial1_h
#define mx28serial1_h

#define MX28_MMX_SERVOS             30
#define MX28_BUFFER_SIZE            32

/** Configuration **/
#if defined(ARBOTIX)
  // no config needed
#elif defined(SERVO_STIK) || defined(ARBOTIX2)
  #define MX_RX_SWITCHED
  #define SET_RX_WR (PORTC |= 0x40)
  #define SET_MX_WR (PORTC |= 0x80)
  #define SET_RX_RD (PORTC = (PORTC & 0xBF) | 0x80)
  #define SET_MX_RD (PORTC = (PORTC & 0x7F) | 0x40)
  #define INIT_MX_RX DDRC |= 0xC0; PORTC |= 0xC0
#elif defined(ARBOTIX_1280)
  #define MX_RX_SWITCHED
  #define SET_RX_WR (PORTG |= 0x08)
  #define SET_MX_WR (PORTG |= 0x10)
  #define SET_RX_RD (PORTG = (PORTG&0xE7) | 0x10 )
  #define SET_MX_RD (PORTG = (PORTG&0xE7) | 0x08 )
  #define INIT_MX_RX DDRG |= 0x18; PORTG |= 0x18
#endif

/** EEPROM AREA **/
#define MX_MODEL_NUMBER_L           0
#define MX_MODEL_NUMBER_H           1
#define MX_VERSION                  2
#define MX_ID                       3
#define MX_BAUD_RATE                4
#define MX_RETURN_DELAY_TIME        5
#define MX_CW_ANGLE_LIMIT_L         6
#define MX_CW_ANGLE_LIMIT_H         7
#define MX_CCW_ANGLE_LIMIT_L        8
#define MX_CCW_ANGLE_LIMIT_H        9
#define MX_SYSTEM_DATA2             10
#define MX_LIMIT_TEMPERATURE        11
#define MX_DOWN_LIMIT_VOLTAGE       12
#define MX_UP_LIMIT_VOLTAGE         13
#define MX_MMX_TORQUE_L             14
#define MX_MMX_TORQUE_H             15
#define MX_RETURN_LEVEL             16
#define MX_ALARM_LED                17
#define MX_ALARM_SHUTDOWN           18
#define MX_OPERATING_MODE           19
#define MX_DOWN_CALIBRATION_L       20
#define MX_DOWN_CALIBRATION_H       21
#define MX_UP_CALIBRATION_L         22
#define MX_UP_CALIBRATION_H         23
/** RAM AREA **/
#define MX_TORQUE_ENABLE            24
#define MX_LED                      25
#define MX_CW_COMPLIANCE_MARGIN     26
#define MX_CCW_COMPLIANCE_MARGIN    27
#define MX_CW_COMPLIANCE_SLOPE      28
#define MX_CCW_COMPLIANCE_SLOPE     29
#define MX_GOAL_POSITION_L          30
#define MX_GOAL_POSITION_H          31
#define MX_GOAL_SPEED_L             32
#define MX_GOAL_SPEED_H             33
#define MX_TORQUE_LIMIT_L           34
#define MX_TORQUE_LIMIT_H           35
#define MX_PRESENT_POSITION_L       36
#define MX_PRESENT_POSITION_H       37
#define MX_PRESENT_SPEED_L          38
#define MX_PRESENT_SPEED_H          39
#define MX_PRESENT_LOAD_L           40
#define MX_PRESENT_LOAD_H           41
#define MX_PRESENT_VOLTAGE          42
#define MX_PRESENT_TEMPERATURE      43
#define MX_REGISTERED_INSTRUCTION   44
#define MX_PAUSE_TIME               45
#define MX_MOVING                   46
#define MX_LOCK                     47
#define MX_PUNCH_L                  48
#define MX_PUNCH_H                  49

#define MX_START                    255
#define MX_RESET_LENGTH             2
#define MX_RESET                    6


/** Status Return Levels **/
#define MX_RETURN_NONE              0
#define MX_RETURN_READ              1
#define MX_RETURN_ALL               2

/** Instruction Set **/
#define MX_PING                     1
#define MX_READ_DATA                2
#define MX_WRITE_DATA               3
#define MX_REG_WRITE                4
#define MX_ACTION                   5
#define MX_RESET                    6
#define MX_SYNC_WRITE               131

/** Error Levels **/
#define ERR_NONE                    0
#define ERR_VOLTAGE                 1
#define ERR_ANGLE_LIMIT             2
#define ERR_OVERHEATING             4
#define ERR_RANGE                   8
#define ERR_CHECKSUM                16
#define ERR_OVERLOAD                32
#define ERR_INSTRUCTION             64

/** AX-S1 **/
#define MX_LEFT_IR_DATA             26
#define MX_CENTER_IR_DATA           27
#define MX_RIGHT_IR_DATA            28
#define MX_LEFT_LUMINOSITY          29
#define MX_CENTER_LUMINOSITY        30
#define MX_RIGHT_LUMINOSITY         31
#define MX_OBSTACLE_DETECTION       32
#define MX_BUZZER_INDEX             40

#define MX_BD_LENGTH                4

void mx28Init(long baud);
void mx28InitPin(unsigned int commPin, long baud);

void setTXall();     // for sync write
void setTX(int id);
void setRX(int id);

void mx28write(unsigned char data);
void mx28writeB(unsigned char data);
int mx28setID(int CurrentID, int NewID);

void mx28SetRegisterID(int id, int regstart, int data);
void mx28SetRegister2(int id, int regstart, unsigned int data);
void mx28SetRegister4(int id, int regstart, unsigned int data1, unsigned int data2);
void mx28SetBR(int id, long baud);
void mx28reset(unsigned char ID);

int mx28ReadPacket(int length);
int mx28GetRegister(int id, int regstart, int length);
void mx28SetRegister(int id, int regstart, int data);
int mx28GetLastError();

extern unsigned char MX_rx_buffer[MX28_BUFFER_SIZE];
extern unsigned char MX_tx_buffer[MX28_BUFFER_SIZE];
extern unsigned char MX_rx_int_buffer[MX28_BUFFER_SIZE];
#if defined(MX_RX_SWITCHED)
// Need to stow type of servo (which bus it's on)
extern unsigned char dynamixel_bus_config[MX28_MMX_SERVOS];
#endif

#define mx28SetPosition(id, pos) (mx28SetRegister2(id, MX_GOAL_POSITION_L, pos))
#define mx28SetPositionSpeed(id, pos, speed) (mx28SetRegister2(id, MX_GOAL_POSITION_L, pos));(mx28SetRegister2(id, MX_GOAL_SPEED_L, speed))
#define mx28GetPosition(id) (mx28GetRegister(id, MX_PRESENT_POSITION_L, 2))
#define mx28TorqueOn(id) (mx28SetRegister(id, MX_TORQUE_ENABLE, 1))
#define mx28Relax(id) (mx28SetRegister(id, MX_TORQUE_ENABLE, 0))

#define mx28GetLeftIRData(id) (mx28GetRegister(id, MX_LEFT_IR_DATA))
#define mx28GetCenterIRData(id) (mx28GetRegister(id, MX_CENTER_IR_DATA))
#define mx28GetRightIRData(id) (mx28GetRegister(id, MX_RIGHT_IR_DATA))
#define mx28GetObstacles(id) (mx28GetRegister(id, OBSTACLE_DETECTION))

#define mx28PlayTone(id, note) (mx28SetRegister(id, MX_BUZZER_INDEX, note))

#endif
