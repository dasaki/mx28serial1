/*****************************************************************

8-oct-2013
David Sanz Kirbis
using modified bioloid ax12 library to setup your MX-28 servos
with a Leonardo board and Serial1 through an 74ls241

for the hardware setup see:
http://savageelectronics.blogspot.com.es/2011/01/arduino-y-dynamixel-ax-12.html

for original library see:
http://forums.trossenrobotics.com/showthread.php?5812-Help-with-controlling-an-Dynamixel-MX-28T-with-an-Arduino-Uno

instructions:
1) plug your brand new MX-28
2) to change the id set the "id" variable to the desired number and uncomment
the instruction: mx28SetRegisterID(1,3,id);
3) to set the baudrate to 1000000, uncomment the instruction mx28SetBR(id,1000000);
After changing the baudrate you will have to re-connect with the new baudrate

Disclaimer: this code is provided as-is, without any warranties. Use it at your own risk.

*****************************************************************/
#include <Arduino.h>
#include <mx28serial1.h>

// NEED TO VERIFY (BAUDRATE==57600 brand-new, out-of-box, usually)
#define BAUDRATE    57600
//#define BAUDRATE    1000000


    // AX/DX/RX/EX/(old-MX firmware)
//#define POS_MAX    1023
    // (new-MX firmware)
#define POS_MAX    4095
#define POS_MIN    0
#define CHANGE_ID_REGISTER 3
 
int id = 254;
int newid = 1;
int ledPin = 13;

void setup() 
{
 //Serial.begin(9600);              // Begin Serial Comunication
//while (!Serial) {
//    ; // wait for serial port to connect. Needed for Leonardo only
 // } 
    mx28Init(BAUDRATE); 
    delay(100);
   //Serial.println("MX-28T test");

 //  mx28SetRegisterID(id,CHANGE_ID_REGISTER,newid);// mx28SetRegisterID(currentID, 3, newID); 
 //  mx28SetBR(id,1000000); // set the baudrate
 //  mx28reset(id);       // WARNING!!!
    delay(100);
    
} 
  
 
 
void loop() 
{ 
    mx28SetPosition(id, random(POS_MIN,POS_MAX));
    digitalWrite(ledPin, HIGH);
    delay(1000); 
    digitalWrite(ledPin, LOW);
    delay(1000); 
} 
 
 

