#include <PID_v1.h>

double counter, output, setpoint, t1, t2, counterx1, counterx2, v;

#define outputA 2
#define outputB 3
#define INTERRUPT 0
#include <Stepper.h>

Stepper posstepper(200, 6, 7, 8, 9);
Stepper negstepper(200, 9, 8, 7, 6);
int aState;
int aLastState;

PID frictionPID(&v, &output, &setpoint, 10, 1, 0, DIRECT);

void setup() {

  setpoint = 0;
  Serial.begin(9600);
  frictionPID.SetMode(AUTOMATIC);
  attachInterrupt(digitalPinToInterrupt(2), counter1, CHANGE);
  pinMode (outputA, INPUT);
  pinMode (outputB, INPUT);
  aLastState = digitalRead(outputA);   // Reads the initial state of the outputA
  frictionPID.SetOutputLimits(0, 200);
  posstepper.setSpeed(250);
  negstepper.setSpeed(250);

}

void loop() {
  VelCalc();
  frictionPID.Compute();
  Serial.print(v);
  Serial.print(" \t");
  Serial.println(output);

  if (output > 0.15 || counter < 200) {
    negstepper.setSpeed(250);
    negstepper.step(output * 100);
  }

  else if (output >= 0.10 && output <= 0.15 && counter < 400 ) {
    negstepper.setSpeed(250);
    negstepper.step(output * 150);

  }

  else if (output >= 0.05 && output < 0.10 && counter > 400 ) {
    negstepper.setSpeed(250);
    negstepper.step(output * 150);
  }

  else if (output < 0.5 && counter < 600) {
    negstepper.setSpeed(250);
    negstepper.step(output * 200);
  }
}



void counter1() {
  aState = digitalRead(outputA); // Reads the "current" state of the outputA

  if (aState != aLastState) {  // If the previous and the current state of the outputA are different, that means a Pulse has occured
    if (digitalRead(outputB) != aState) {   // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
      counter --;
    } else {
      counter ++;
    }
  }
  aLastState = aState; //
}

void VelCalc() {
  if (counter >= 1) {
    t1 = millis();
    counterx1 = counter * 1.725;
    /* Serial.print(t1);
      Serial.print("       ");
      Serial.print(counterx1);
    */

    delay(300);
    counterx2 = counter * 1.725;
    t2 = millis();
    /*Serial.print("         ");
      Serial.print(t2);
      Serial.print("       ");
      Serial.print(counterx2);
    */
    v = (float)(counterx2 - counterx1) / (t2 - t1);
    counter = 0;

    /* Serial.print("          ");
      Serial.println(v);
    */
  }
}


