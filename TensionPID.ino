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

#include <Stepper.h>
#include <PID_v1.h>
#include <TimerThree.h>
#include <Servo.h>
#include <IRremote.h>
// *******************************************************************************
//              IR STUFF
const int RECV_PIN = 4;
IRrecv irrecv(RECV_PIN);
decode_results results;
int onoff = 0;
//****************************************************************************************
double  tension, output, setpoint, frictionForce, angle;

Servo esc, esch1, esch2;
int stepcounter = 0;

Stepper lessT(200, 6, 7, 8, 9);
Stepper moreT(200, 9, 8, 7, 6);
PID frictionPID(&tension, &output, &setpoint, .5, 0, 0, DIRECT);

void setup() {
  //  Timer3.initialize();
  //  Timer3.attachInterrupt(control, 10000); //(10 ms period)
  irrecv.enableIRIn();
  irrecv.blink13(true);
  setpoint = 30;
  frictionPID.SetMode(AUTOMATIC);
  frictionPID.SetOutputLimits(-10, 10);
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  Serial.println(onoff);

  if (irrecv.decode(&results)) {
    onoff++;
    irrecv.resume();
  }


  while (onoff == 1) {
    Serial.print("im in the loop");

    if (irrecv.decode(&results)) {
      onoff++;
      irrecv.resume();
    }

    frictionPID.Compute();
    if (Serial1.available()) {
      tension = Serial1.read();
      tension = map(tension, 0, 255, 0, 115);
    }
    Serial.print(tension);
    Serial.print("\t");
    Serial.print(output);
    Serial.print("\t");
    Serial.println(stepcounter);

    if (stepcounter < 400) {
      if (output >= 0) {
        lessT.setSpeed(200);
        moreT.step(output);
        stepcounter = stepcounter + output;
      }
    }


    if (output < 0) {
       if (stepcounter >-400){
      moreT.setSpeed(200);
      moreT.step(-output);
      stepcounter = stepcounter + output;
    }
}

  }
  onoff=0;
  stepcounter=0;
}

void control() {
  frictionPID.Compute();
}


//frictionForce=.00000004*pow(angle, 3)- .00004*pow(angle, 2)+ 0.014*angle-0.5275;


