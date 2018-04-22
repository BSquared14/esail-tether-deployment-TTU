/*****************************************************************
XBee_Serial_Passthrough.ino

Set up a software serial port to pass data between an XBee Shield
and the serial monitor.

Hardware Hookup:
  The XBee Shield makes all of the connections you'll need
  between Arduino and XBee. If you have the shield make
  sure the SWITCH IS IN THE "DLINE" POSITION. That will connect
  the XBee's DOUT and DIN pins to Arduino pins 2 and 3.

*****************************************************************/
// We'll use SoftwareSerial to communicate with the XBee:
#include <SoftwareSerial.h>
// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
//SoftwareSerial XBee(2,3); // RX, TX

float tension;

void setup()
{
  // Set up both ports at 9600 baud. This value is most important
  // for the XBee. Make sure the baud rate matches the config
  // setting of your XBee.
//  XBee.begin(9600);
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop()
{
  if (Serial.available())
  { // If data comes in from serial monitor, send it out to XBee
    Serial1.write(Serial.read());
  }
  if (Serial1.available())
  { // If data comes in from XBee, send it out to serial monitor
//    Serial.println(Serial1.read());
    tension=Serial1.read();
//    tension=map(tension, 0, 255, 0, 11556);
    Serial.println(tension);
}

  
}
