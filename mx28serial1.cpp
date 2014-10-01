/*
  mx28serial1.cpp

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

#include "mx28serial1.h"

/******************************************************************************
 * Hardware Serial Level, this uses the same stuff as Serial1, therefore 
 *  you should not use the Arduino Serial1 library.
 */

unsigned char mx_rx_buffer[MX28_BUFFER_SIZE];
unsigned char mx_tx_buffer[MX28_BUFFER_SIZE];
unsigned char mx_rx_int_buffer[MX28_BUFFER_SIZE];

// making these volatile keeps the compiler from optimizing loops of available()
volatile int mx_rx_Pointer;
volatile int mx_tx_Pointer;
volatile int mx_rx_int_Pointer;
#if defined(MX_RX_SWITCHED)
unsigned char dynamixel_bus_config[MX28_MMX_SERVOS];
#endif

/** helper functions to switch direction of comms */
void setTX(int id){
    bitClear(UCSR1B, RXEN1); 
  #if defined(MX_RX_SWITCHED)
    if(dynamixel_bus_config[id-1] > 0)
        SET_RX_WR;
    else
        SET_MX_WR;   
  #else
    // emulate half-duplex on ArbotiX, ArbotiX w/ RX Bridge
    #ifdef ARBOTIX_WITH_RX
      PORTD |= 0x10;
    #endif   
    bitSet(UCSR1B, TXEN1);
    bitClear(UCSR1B, RXCIE1);
  #endif
    mx_tx_Pointer = 0;
}
void setRX(int id){ 
  #if defined(MX_RX_SWITCHED)
    int i;
    // Need to wait for last byte to be sent before turning the bus around.
    // Check the Transmit complete flag
    while (bit_is_clear(UCSR1A, UDRE1));
    for(i=0; i<UBRR1L*15; i++)    
        asm("nop");
			    if(dynamixel_bus_config[id-1] > 0)
        SET_RX_RD;
    else
        SET_MX_RD;
  #else
    // emulate half-duplex on ArbotiX, ArbotiX w/ RX Bridge
    #ifdef ARBOTIX_WITH_RX
      int i;
      // Need to wait for last byte to be sent before turning the bus around.
      // Check the Transmit complete flag
      while (bit_is_clear(UCSR1A, UDRE1));
      for(i=0; i<25; i++)    
          asm("nop");
      PORTD &= 0xEF;
    #endif 
    bitClear(UCSR1B, TXEN1);
    bitSet(UCSR1B, RXCIE1);
  #endif  
    bitSet(UCSR1B, RXEN1);
    mx_rx_int_Pointer = 0;
    mx_rx_Pointer = 0;
}
// for sync write
void setTXall(){
    bitClear(UCSR1B, RXEN1);    
  #if defined(MX_RX_SWITCHED)
    SET_RX_WR;
    SET_MX_WR;   
  #else
    #ifdef ARBOTIX_WITH_RX
      PORTD |= 0x10;
    #endif
    bitSet(UCSR1B, TXEN1);
    bitClear(UCSR1B, RXCIE1);
  #endif
    mx_tx_Pointer = 0;
}

/** Sends a character out the serial port. */
void mx28write(unsigned char data){
    while (bit_is_clear(UCSR1A, UDRE1));
    UDR1 = data;
}
/** Sends a character out the serial port, and puts it in the tx_buffer */
void mx28writeB(unsigned char data){
    mx_tx_buffer[(mx_tx_Pointer++)] = data; 
    while (bit_is_clear(UCSR1A, UDRE1));
    UDR1 = data;
}
/** We have a one-way recieve buffer, which is reset after each packet is receieved.
    A wrap-around buffer does not appear to be fast enough to catch all bytes at 1Mbps. */
ISR(USART0_RX_vect){
    mx_rx_int_buffer[(mx_rx_int_Pointer++)] = UDR1;
}

/** read back the error code for our latest packet read */
int mx28Error;
int mx28GetLastError(){ return mx28Error; }
/** > 0 = success */
int mx28ReadPacket(int length){
    unsigned long ulCounter;
    unsigned char offset, blength, checksum, timeout;
    unsigned char volatile bcount; 

    offset = 0;
    timeout = 0;
    bcount = 0;
    while(bcount < length){
        ulCounter = 0;
        while((bcount + offset) == mx_rx_int_Pointer){
            if(ulCounter++ > 1000L){ // was 3000
                timeout = 1;
                break;
            }
        }
        if(timeout) break;
        mx_rx_buffer[bcount] = mx_rx_int_buffer[bcount + offset];
        if((bcount == 0) && (mx_rx_buffer[0] != 0xff))
            offset++;
        else if((bcount == 2) && (mx_rx_buffer[2] == 0xff))
            offset++;
        else
            bcount++;
    }

    blength = bcount;
    checksum = 0;
    for(offset=2;offset<bcount;offset++)
        checksum += mx_rx_buffer[offset];
    if((checksum%256) != 255){
        return 0;
    }else{
        return 1;
    }
}

/** initializes serial1 transmit at baud, 8-N-1 */
void mx28Init(long baud){
    UBRR1H = (F_CPU / (8 * baud) - 1 ) >> 8;
    UBRR1L = (F_CPU / (8 * baud) - 1 );
    bitSet(UCSR1A, U2X1);
    mx_rx_int_Pointer = 0;
    mx_rx_Pointer = 0;
    mx_tx_Pointer = 0;
#if defined(MX_RX_SWITCHED)
    INIT_MX_RX;
    bitSet(UCSR1B, TXEN1);
    bitSet(UCSR1B, RXEN1);
    bitSet(UCSR1B, RXCIE1);
#else
  #ifdef ARBOTIX_WITH_RX
    DDRD |= 0x10;   // Servo B = output
    PORTD &= 0xEF;  // Servo B low
  #endif
    // set RX as pull up to hold bus to a known level
    PORTD |= (1<<2);
    // enable rx
    setRX(0);
#endif

}

void mx28InitPin(unsigned int commPin, long baud) {
    UBRR1H = (F_CPU / (8 * baud) - 1 ) >> 8;
    UBRR1L = (F_CPU / (8 * baud) - 1 );
    bitSet(UCSR1A, U2X1);
    mx_rx_int_Pointer = 0;
    mx_rx_Pointer = 0;
    mx_tx_Pointer = 0;
#if defined(MX_RX_SWITCHED)
    INIT_MX_RX;
    bitSet(UCSR1B, TXEN1);
    bitSet(UCSR1B, RXEN1);
    bitSet(UCSR1B, RXCIE1);
#else
  #ifdef ARBOTIX_WITH_RX
    DDRD |= commPin;   // Servo B = output
    PORTD &= 0xEF;  // Servo B low
  #endif
    // set RX as pull up to hold bus to a known level
    PORTD |= (1<<2);
    // enable rx
    setRX(0);
#endif
}

/******************************************************************************
 * Packet Level
 */

/** Read register value(s) */
int mx28GetRegister(int id, int regstart, int length){  
    setTX(id);
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
    int checksum = ~((id + 6 + regstart + length)%256);
    mx28writeB(0xFF);
    mx28writeB(0xFF);
    mx28writeB(id);
    mx28writeB(4);    // length
    mx28writeB(MX_READ_DATA);
    mx28writeB(regstart);
    mx28writeB(length);
    mx28writeB(checksum);  
    setRX(id);    
    if(mx28ReadPacket(length + 6) > 0){
        mx28Error = mx_rx_buffer[4];
        if(length == 1)
            return mx_rx_buffer[5];
        else
            return mx_rx_buffer[5] + (mx_rx_buffer[6]<<8);
    }else{
        return -1;
    }
}
/*
int mx28setID (int CurrentID, int NewID) {
 
    char data[1];
    data[0] = NewID;
 
#ifdef MX28_DEBUG
    printf("Setting ID from 0x%x to 0x%x\n",CurrentID,NewID);
#endif
 
    return (mx28writeID(CurrentID, MX_ID, 1, data));
 
}*/
/* Set the value of a single-byte register. */
void mx28SetRegister(int id, int regstart, int data){
    setTX(id);    
    int checksum = ~((id + 4 + MX_WRITE_DATA + regstart + (data&0xff)) % 256);
    mx28writeB(0xFF);
    mx28writeB(0xFF);
    mx28writeB(id);
    mx28writeB(4);    // length
    mx28writeB(MX_WRITE_DATA);
    mx28writeB(regstart);
    mx28writeB(data&0xff);
    // checksum = 
    mx28writeB(checksum);
    setRX(id);
    //mx28ReadPacket();
}
void mx28SetRegisterID(int id, int regstart, int data){
    setTX(id);    
    int checksum = ~((id + 4 + MX_WRITE_DATA + regstart + (data&0xff)) % 256);
    mx28writeB(0xFF);
    mx28writeB(0xFF);
    mx28writeB(id);
    mx28writeB(4);    // length
    mx28writeB(MX_WRITE_DATA);
    mx28writeB(regstart);
    mx28writeB(data&0xff);
    // checksum = 
    mx28writeB(checksum);
    setRX(data);
    //mx28ReadPacket();
}
/* Set the value of a double-byte register. */
void mx28SetRegister2(int id, int regstart, unsigned int data){
    setTX(id);    
    int checksum = ~((id + 5 + MX_WRITE_DATA + regstart + (data&0xFF) + ((data&0xFF00)>>8)) % 256);
    mx28writeB(0xFF);
    mx28writeB(0xFF);
    mx28writeB(id);
    mx28writeB(5);    // length
    mx28writeB(MX_WRITE_DATA);
    mx28writeB(regstart);
    mx28writeB(data&0xff);
    mx28writeB((data&0xff00)>>8);
    // checksum = 
    mx28writeB(checksum);
    setRX(id);
    //mx28ReadPacket();
}

void mx28SetRegister4(int id, int regstart, unsigned int data1, unsigned int data2)
{
    setTX(id);    
    int checksum = ~((id + 7 + MX_WRITE_DATA + regstart + (data1&0xFF) + ((data1&0xFF00)>>8)+ (data2&0xFF) + ((data2&0xFF00)>>8)) % 256);
    mx28writeB(0xFF);
    mx28writeB(0xFF);
    mx28writeB(id);
    mx28writeB(7);    // length
    mx28writeB(MX_WRITE_DATA);
    mx28writeB(regstart);
    mx28writeB(data1&0xff);
    mx28writeB((data1&0xff00)>>8);
    mx28writeB(data2&0xff);
    mx28writeB((data2&0xff00)>>8);
    // checksum = 
    mx28writeB(checksum);
    setRX(id);
    //mx28ReadPacket();
}

/* Set the baudrate. */
void mx28SetBR(int id, long baud){

    unsigned char Baud_Rate = (2000000/baud) - 1;
    mx28SetRegister(id, MX_BAUD_RATE, Baud_Rate);
/*    setTX(id);    
    unsigned char Baud_Rate = (2000000/baud) - 1;
    int checksum = (~(id + MX_BD_LENGTH + MX_WRITE_DATA + MX_BAUD_RATE + Baud_Rate))&0xFF;
    mx28writeB(0xFF);
    mx28writeB(0xFF);
    mx28writeB(id);
    mx28writeB(7);    // length
    mx28writeB(MX_WRITE_DATA);
    mx28writeB(regstart);
    mx28writeB(baud&0xff);
    mx28writeB((baud&0xff00)>>8);
    mx28writeB((baud&0xff0000)>>16);
    mx28writeB((baud&0xff000000)>>24);   
    // checksum = 
    mx28writeB(checksum);
    setRX(id);
    //mx28ReadPacket();
*/
}

// general write?
// general sync write?
/*
int mx28writeID(int ID, int start, int bytes, char* data, int flag) {
// 0xff, 0xff, ID, Length, Intruction(write), Address, Param(s), Checksum
 
    char TxBuf[16];
    char sum = 0;
    char Status[6];
 
#ifdef MX28_WRITE_DEBUG
    printf("\nwrite(%d,0x%x,%d,data,%d)\n",ID,start,bytes,flag);
#endif
 
    // Build the TxPacket first in RAM, then we'll send in one go
#ifdef MX28_WRITE_DEBUG
    printf("\nInstruction Packet\n  Header : 0xFF, 0xFF\n");
#endif
 
    TxBuf[0] = 0xff;
    TxBuf[1] = 0xff;
 
    // ID
    TxBuf[2] = ID;
    sum += TxBuf[2];
 
#ifdef MX28_WRITE_DEBUG
    printf("  ID : %d\n",TxBuf[2]);
#endif
 
    // packet Length
    TxBuf[3] = 3+bytes;
    sum += TxBuf[3];
 
#ifdef MX28_WRITE_DEBUG
    printf("  Length : %d\n",TxBuf[3]);
#endif
 
    // Instruction
    if (flag == 1) {
        TxBuf[4]=0x04;
        sum += TxBuf[4];
    } else {
        TxBuf[4]=0x03;
        sum += TxBuf[4];
    }
 
#ifdef MX28_WRITE_DEBUG
    printf("  Instruction : 0x%x\n",TxBuf[4]);
#endif
 
    // Start Address
    TxBuf[5] = start;
    sum += TxBuf[5];
 
#ifdef MX28_WRITE_DEBUG
    printf("  Start : 0x%x\n",TxBuf[5]);
#endif
 
    // data
    for (char i=0; i<bytes ; i++) {
        TxBuf[6+i] = data[i];
        sum += TxBuf[6+i];
 
#ifdef MX28_WRITE_DEBUG
        printf("  Data : 0x%x\n",TxBuf[6+i]);
#endif
 
    }
 
    // checksum
    TxBuf[6+bytes] = 0xFF - sum;
 
#ifdef MX28_WRITE_DEBUG
    printf("  Checksum : 0x%x\n",TxBuf[6+bytes]);
#endif
 
    // Transmit the packet in one burst with no pausing
    for (int i = 0; i < (7 + bytes) ; i++) {
        mx28writeB(TxBuf[i]);
    }
 
    // Wait for data to transmit
    wait (0.00002);
 
    // make sure we have a valid return
    Status[4]=0x00;
 
    // we'll only get a reply if it was not broadcast
    if (_ID!=0xFE) {
 
 
        // response packet is always 6 bytes
        // 0xFF, 0xFF, ID, Length Error, Param(s) Checksum
        // timeout is a little more than the time to transmit
        // the packet back, i.e. 60 bit periods, round up to 100
        int timeout = 0;
        int plen = 0;
        while ((timeout < 100) && (plen<6)) {
 
            if (_mx28.readable()) {
                Status[plen] = _mx28.getc();
                plen++;
                timeout = 0;
            }
 
            // wait for the bit period
            wait (1.0/_baud);
            timeout++;
        }
 
 
        // Build the TxPacket first in RAM, then we'll send in one go
#ifdef MX28_WRITE_DEBUG
        printf("\nStatus Packet\n  Header : 0x%X, 0x%X\n",Status[0],Status[1]);
        printf("  ID : %d\n",Status[2]);
        printf("  Length : %d\n",Status[3]);
        printf("  Error : 0x%x\n",Status[4]);
        printf("  Checksum : 0x%x\n",Status[5]);
#endif
 
 
    }
 
    return(Status[4]); // return error code
}*/

void mx28reset(unsigned char ID)
{
	int Checksum = (~(ID + MX_RESET_LENGTH + MX_RESET))&0xFF;
	//int checksum = ~((id + 4 + MX_WRITE_DATA + regstart + (data&0xff)) % 256);

	setTX(ID);
        mx28writeB(MX_START);                     
	mx28writeB(MX_START);
	mx28writeB(ID);
	mx28writeB(MX_RESET_LENGTH);
	mx28writeB(MX_RESET);    
	mx28writeB(Checksum);
	//wait (0.00002);
        setRX(ID);
}
