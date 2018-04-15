/* VernierLibTutorialAnalogRead (v2017)
   This sketch reads a data point from a Vernier Analog (BTA)
   sensor once every half second and prints the sensor reading
   with units to the Serial Monitor.

   Plug the sensor into the Analog 1 port on the Vernier Arduino
   Interface Shield or into an Analog Protoboard Adapter wired
   to Arduino pin A0.
*/

#include "VernierLib.h" //include Vernier functions in this sketch
VernierLib Vernier; //create an instance of the VernierLib library

float tension;//create global variable to store sensor reading


#include <SoftwareSerial.h>
// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
SoftwareSerial XBee(2, 3); // RX, TX

void setup() {
  Serial.begin(9600); //setup communication to display
  Vernier.autoID(); //identify the sensor being used
  XBee.begin(9600);
}

void loop() {
  tension = Vernier.readSensor(); //read one data value
  tension = (tension - .72)*1000;
  tension=map(tension, 0,11530, 0, 255);
  delay(100);
  XBee.write(tension);
  Serial.println(tension);

}

//Vernier pin _____color________arduino pin
//Vpin1----------> yellow ----->
//Vpin2----------> green ------>Ground
//Vpin3----------> blue ------->Analog 4
//Vpin4----------> purple ----->Analog 5
//Vpin5----------> grey ------->VCC
//Vpin6----------> white ------>Analog 0


