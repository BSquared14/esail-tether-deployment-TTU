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
    v1.03           Everything changed
    Next...           PID Tuning, 1D deployment with 2 CubeSat, ROS communication, 2D Deployment
    
Vernier pin _____color________arduino pin
    Vpin1-----> yellow ------->
    Vpin2-----> green -------->Ground
    Vpin3-----> blue --------->Analog 4
    Vpin4-----> purple ------->Analog 5
    Vpin5-----> grey --------->VCC
    Vpin6-----> white -------->Analog 1
*/

#include <Timer3.h>
#include <Stepper.h>
#include <PID_v1.h>
#include <TimerThree.h>

double tension, output, setpoint, frictionForce, angle;

Stepper lessT(200, 6, 7, 8, 9);
Stepper moreT(200, 9, 8, 7, 6);
PID frictionPID(&tension, &output, &setpoint, 1, 0, 0, DIRECT);

void setup() {
  Timer3.initialize();
  Timer3.attachInterrupt(control, 10000); //(10 ms period)
  setpoint = 10;
  frictionPID.SetMode(AUTOMATIC);
  frictionPID.SetOutputLimits(-5, 5);
  Serial.begin(9600);
  Serial1.begin(9600);

}

void loop() {

  if (Serial1.available() >= 0) {
    tension = Serial1.read();
    if (tension > 60)
      tension = 0;
  }
  Serial.print(tension);
  Serial.print("\t");
  Serial.println(output);
  delay(100);

  if (output >= 0) {
    moreT.setSpeed(200);
    moreT.step(output);
  }

  if (output < 0) {
    lessT.setSpeed(200);
    lessT.step(-output);
  }

}

void control() {
  frictionPID.Compute();
}

//frictionForce=.00000004*pow(angle, 3)- .00004*pow(angle, 2)+ 0.014*angle-0.5275;
