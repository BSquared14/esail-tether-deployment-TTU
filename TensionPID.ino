/*
    Tension_Control v1.02
      HERTS Tether Deployment Model
      TTU Mechatronics - 4/9/2018
      Ben Brandt, Seth Shearman
      Thanks to Tristin Hill, Dr. Stephen Canfield, and the makers of the libraries used

      This Arduino maintains a tether tension setpoint read from the ROS Master.
      Stepper angle at friction device is modulated to achieve this, and is mirrored by the tension control in another arduino.

    Changelog:
    v1.00           drop test, hardware initialization, encoder reading in interrupt, stepper functionality
    v1.01           quadrature encoding without missing counts. New encoder.
    v1.02           Streamlined velocity calculation. 1D Serial Communication. 1D 1 CubeSat Deployment
    Next...           PID Tuning, 1D deployment with 2 CubeSat, ROS communication, 2D Deployment
    
Vernier pin _____color________arduino pin
    Vpin1-----> yellow ------->
    Vpin2-----> green -------->Ground
    Vpin3-----> blue --------->Analog 4
    Vpin4-----> purple ------->Analog 5
    Vpin5-----> grey --------->VCC
    Vpin6-----> white -------->Analog 1
*/

#include <VernierLib.h>
#include <TimerThree.h>
#include <PID_v1.h>
#include <Stepper.h>

VernierLib Vernier;

volatile double tension, output, setpoint, counter;
volatile double currentTime = 0, lastTime = 0;
#define outputA 2
#define outputB 3
#define INTERRUPT 0
#define INTERRUPT 1
bool aState, bState;

Stepper posstepper(200, 6, 7, 8, 9);
Stepper negstepper(200, 9, 8, 7, 6);
PID frictionDevice(&tension, &output, &setpoint, 1, 0, 1, DIRECT);

void setup() {

  Timer3.initialize();
  Timer3.attachInterrupt(control, 10000);                           //Timer1 overflows to trigger the interrupt every 0.01s
  pinMode (outputA, INPUT);
  pinMode (outputB, INPUT);
    
  Vernier.autoID();
    
  frictionDevice.SetMode(AUTOMATIC);
  frictionDevice.SetSampleTime(100);
  frictionDevice.SetOutputLimits(-100, 100); //~270 maximum

  setpoint = 100;
  Serial.begin (9600);
  while (!Serial) {//Wait for USB serial to connect
  }
}

void loop() {
   
  Serial.println(tension);
    
  //  Serial.print("\t");
  //  Serial.print(output);
  //  Serial.print("\t");
  //  Serial.println(setpoint);

  tension = Vernier.readSensor();
    
  if (output > 0) {                 // evaluate stepping for smooth output
    posstepper.setSpeed(150);
    posstepper.step(output);
  }
  else if (output < 0) {
    negstepper.setSpeed(150);
    negstepper.step(-output);
  }
}

void control() {//Triggered by timer overflow. Must refresh slowly enough to completely run or will "lock" the rest of the program, never returning to the main loop.
  
  frictionDevice.Compute();
}

