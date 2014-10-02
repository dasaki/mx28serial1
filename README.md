mx28serial1
===========

Using modified bioloid ax12 library to setup your MX-28 servos
with a Leonardo board and Serial1 through an 74ls241

for the hardware setup see:
http://savageelectronics.blogspot.com.es/2011/01/arduino-y-dynamixel-ax-12.html

for original library see:
http://forums.trossenrobotics.com/showthread.php?5812-Help-with-controlling-an-Dynamixel-MX-28T-with-an-Arduino-Uno

Instructions:
1) plug your brand new MX-28
2) to change the id set the "id" variable to the desired number and uncomment
the instruction: mx28SetRegisterID(1,3,id);
3) to set the baudrate to 1000000, uncomment the instruction mx28SetBR(id,1000000);
After changing the baudrate you will have to re-connect with the new baudrate

Disclaimer: this code is provided as-is, without any warranties. Use it at your own risk.
