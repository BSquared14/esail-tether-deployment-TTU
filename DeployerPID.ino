/*
    Deployment_Control v1.02
      HERTS Tether Deployment Model
      TTU Mechatronics - 4/9/2018
      Ben Brandt, Seth Shearman
      Thanks to Tristin Hill, Dr. Stephen Canfield, and the makers of the libraries used

      This Arduino maintains a tether deployment velocity read from the ROS Master.
      Deployment fan thrust is modulated to achieve this.

    Changelog:
    v1.00           drop test, hardware initialization, encoder reading in interrupt, stepper functionality
    v1.01           quadrature encoding without missing counts. New encoder.
    v1.02           Streamlined velocity calculation. 1D Serial Communication. 1D 1 CubeSat Deployment
    Next...           PID Tuning, 1D deployment with 2 CubeSat, ROS communication, 2D Deployment
*/

#include <TimerThree.h>
#include <PID_v1.h>
#include <Servo.h>

volatile double velocity, output, setpoint, counter, counterx1, counterx2, t;
int minimum = 1235;
int maxaximum = 1600;

#define outputA 2
#define outputB 3
#define INTERRUPT 0
#define INTERRUPT 1
bool aState, bState;

Servo esc;
Servo esch1;
Servo esch2;
PID frictionDevice(&velocity, &output, &setpoint, 100, 5, 1, P_ON_M, DIRECT);

void setup() {


  Timer3.initialize();
  Timer3.attachInterrupt(control, 10000);                           //Timer1 overflows to trigger the interrupt every 0.01s
  attachInterrupt(digitalPinToInterrupt(2), encoderA, CHANGE);
  pinMode (outputA, INPUT);
  pinMode (outputB, INPUT);

  frictionDevice.SetMode(AUTOMATIC);
  //frictionDevice.SetSampleTime(100);
  frictionDevice.SetOutputLimits(-1, 363); //~270 maximum

  setpoint = .1;
  Serial.begin (9600);
  esc.attach(11);  //Specify here the pin number on which the signal pin of ESC is connected.
  esch1.attach(12);
  esch2.attach(13);
  esc.writeMicroseconds(1000);   //ESC arm command. ESCs won't start unless input speed is less during initialization.
  esch1.writeMicroseconds(1000);
  esch2.writeMicroseconds(1000);
  delay(3000);              // delay time to initialize esc
}


void loop() {
  esch1.writeMicroseconds(1450);
  esch2.writeMicroseconds(1400);
//
//  t = millis();
//  if (t > 20000)
//    setpoint = 0;

  velCalc();
  Serial.print(velocity);
  Serial.print("\t");
  Serial.print(setpoint);
  Serial.print("\t");
  Serial.println(output);

  esc.writeMicroseconds(minimum + output);

}

//interrupts are disabled during a triggered interrupt's subroutine. millis() is based on an interrupt, so it is actually flagged and executed after the current interrupt is finished.
//The time period from the counter incrementing and the time actually being recorded is about 3 microseconds.
void encoderA() {//Read both bytes, trigger on A change. If both bytes are different, increment.
  aState = (PINE &= B00010000);
  bState = (PINE &= B00100000);
  if (aState && (!bState)) {                    //if A:1 B:0
    counter++;
  }
  else if (aState && bState) {                  //if A:1 B:1
    counter--;
  }
}

void control() {//Triggered by timer overflow. Must refresh slowly enough to completely run or will "lock" the rest of the program, never returning to the main loop.
  frictionDevice.Compute();
}

void velCalc() {           //in .1 m/s
  counterx1 = counter;
  delay(50);
  counterx2 = counter;
  velocity = (counterx2 - counterx1) / 50;
}
