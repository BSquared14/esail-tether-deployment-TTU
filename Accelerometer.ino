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
#include<Wire.h>
#include<Servo.h>
#include <IRremote.h>

const int MPU_addr = 0x68; // I2C address of the MPU-6050
double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, output, setpoint;
Servo escLeft, escRight, esch1, esch2;
int leftspeed, rightspeed, i, AcYavg;
double AccelerationY, AccelerationZ, RotationX;
int minimum = 1225;

// *******************************************************************************
//              IR STUFF
const int RECV_PIN = 10;
IRrecv irrecv(RECV_PIN);
decode_results results;
int value;
int onoff = 0;
//****************************************************************************************

PID straightPID(&RotationX, &output, &setpoint, .2, 0, 0, DIRECT);

void setup() {
  Timer3.initialize();
  Timer3.attachInterrupt(control, 10000);

  straightPID.SetMode(AUTOMATIC);
  straightPID.SetOutputLimits(-100, 100);
  setpoint = 0;
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  escLeft.attach(11);
  escRight.attach(12);
  escLeft.attach(5);
  escRight.attach(6);
  esch1.writeMicroseconds(1000); //hoverboard 1
  esch2.writeMicroseconds(1000); //hoverboard 2
  escRight.writeMicroseconds(1500);
  escLeft.writeMicroseconds(1500);
  delay(2000);
  escRight.writeMicroseconds(1100);
  escLeft.writeMicroseconds(1100);
  delay(2000);
}

void loop() {
  if (irrecv.decode(&results)) {
    onoff++;
    if (onoff == 1) {
      function();
    }

    else if (onoff == 2) {
      onoff = 0;
    }
    irrecv.resume();
  }
}
  

void control() {
  straightPID.Compute();
}
void function(){

Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  AccelerationY = AcY + 290; // makes the running average around 0
  AccelerationZ = AcZ + 370; // makes the running average around 0
  RotationX = GyY + 360;

  if ((RotationX > -1000) && (RotationX < 1000)) {
    RotationX = 0;
  }
  Serial.print(RotationX);
  Serial.print("\t");
  Serial.print(output);
  Serial.print("\t");
  Serial.print(leftspeed);
  Serial.print("\t");
  Serial.println(rightspeed);

  if (output > 0) {
    leftspeed = 1325 + output;
    rightspeed = 1000;
    escLeft.writeMicroseconds(leftspeed);
    escRight.writeMicroseconds(rightspeed);
    delay(1000);
  }

  if (output <= 0 ) {
    rightspeed = 1150 -  output;
    leftspeed = 1300;
    escRight.writeMicroseconds(rightspeed);
    escLeft.writeMicroseconds(leftspeed);
    delay(1000);
  }
}
