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
#include <Stepper.h>

volatile double velocity, output, setpoint, counter;
volatile double currentTime = 0, lastTime = 0;
#define outputA 2
#define outputB 3
#define INTERRUPT 0
#define INTERRUPT 1
bool aState, bState;

Stepper posstepper(200, 6, 7, 8, 9);
Stepper negstepper(200, 9, 8, 7, 6);
PID frictionDevice(&velocity, &output, &setpoint, 1, 0, 1, DIRECT);

void setup() {

  Timer3.initialize();
  Timer3.attachInterrupt(control, 10000);                           //Timer1 overflows to trigger the interrupt every 0.01s
  attachInterrupt(digitalPinToInterrupt(2), encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), encoderB, CHANGE);
  pinMode (outputA, INPUT);
  pinMode (outputB, INPUT);

  frictionDevice.SetMode(AUTOMATIC);
  frictionDevice.SetSampleTime(100);
  frictionDevice.SetOutputLimits(-100, 100); //~270 maximum

  setpoint = 100;
  Serial.begin (9600);
  while (!Serial) {//Wait for USB serial to connect
  }
}

void loop() {

  Serial.println(velocity);
  //  Serial.print("\t");
  //  Serial.println(velocity);
  //  Serial.print("\t");
  //  Serial.print(output);
  //  Serial.print("\t");
  //  Serial.println(setpoint);


//  velocity = 1000 / (currentTime - lastTime);  //re-evaluate this. Counts per second.
//  lastTime = currentTime;

  if (output > 0) {// Insert Deployment Fans PWM control here
  
  }
  else if (output < 0) {
  
  }
}

//interrupts are disabled during a triggered interrupt's subroutine. millis() is based on an interrupt, so it is actually flagged and executed after the current interrupt is finished.
//The time period from the counter incrementing and the time actually being recorded is about 3 microseconds.
void encoderA() {//Read both bytes, trigger on A change. If both bytes are different, increment.
  aState = (PINE &= B00010000);
  bState = (PINE &= B00100000);
  if (aState && (!bState)) {                    //if A:1 B:0
    counter++;
    currentTime = millis();
  }
  else if (aState && bState) {                  //if A:1 B:1
    counter--;
    currentTime = millis();
  }
  else if ((!aState) && bState) {               //if A:0 B:1
    counter++;
    currentTime = millis();
  }
  else if ((!aState) && (!bState)) {            //if A:0 B:0
    counter--;
    currentTime = millis();
  }
}

void encoderB() {//Read both bytes, trigger on B change. If both bytes are the same, increment.
  aState = (PINE &= B00010000);
  bState = (PINE &= B00100000);
  if (bState && aState) {                       //if B:1 A:1
    counter++;
    currentTime = millis();
  }
  else if (bState && (!aState)) {               //if B:1 A:0
    counter--;
    currentTime = millis();
  }
  else if ((!bState) && (!aState)) {            //if B:0 A:0
    counter++;
    currentTime = millis();
  }
  else if ((!bState) && (aState)) {             //if B:0 A:1
    counter--;
    currentTime = millis();
  }

}

void control() {//Triggered by timer overflow. Must refresh slowly enough to completely run or will "lock" the rest of the program, never returning to the main loop.
  
  frictionDevice.Compute();
}

