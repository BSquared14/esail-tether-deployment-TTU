#include <VernierLib.h>
#include <PID_v1.h>
#include <Servo.h>
double tension, output, setpoint;
VernierLib Vernier;
PID tensionPID(&tension, &output, &setpoint, 10, 0, 0, DIRECT);
Servo esc;
int goodspeed = 1150;


void setup() {
  setpoint = 5;
  tensionPID.SetMode(AUTOMATIC);
  Serial.begin(9600);
  Vernier.autoID();
  esc.attach(12);  //Specify here the pin number on which the signal pin of ESC is connected.
  esc.writeMicroseconds(1000);   //ESC arm command. ESCs won't start unless input speed is less during initialization.
  delay(3000);            //ESC initialization delay.
  tensionPID.SetOutputLimits(-50, 50);
}

void loop() {
  tension = Vernier.readSensor(); //read one data value
  tension = tension - .79;
  tensionPID.Compute();
  Serial.print(tension);
  Serial.print(" \t");
  Serial.println(output);
  Serial.print("           ");
  Serial.println(goodspeed);

  if (output < 0) {
    goodspeed = goodspeed + output;
    esc.writeMicroseconds(goodspeed);
  }
  else {
    goodspeed = goodspeed + output;
    esc.writeMicroseconds(goodspeed);

    if (goodspeed > 2000) {
      goodspeed = 2000 - 50;
    }
  }


}

//Vernier pin _____color________arduino pin
//Vpin1----------> yellow ----->
//Vpin2----------> green ------>Ground
//Vpin3----------> blue ------->Analog 4
//Vpin4----------> purple ----->Analog 5
//Vpin5----------> grey ------->VCC
//Vpin6----------> white ------>Analog 1


