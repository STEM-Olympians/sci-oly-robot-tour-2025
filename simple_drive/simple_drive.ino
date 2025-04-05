#include <Wire.h>
#include <MPU6050.h>
#include "MPU6050_6Axis_MotionApps612.h"

// MPU!!!
MPU6050 mpu;

#define LeftMotFwd 10  // white
#define LeftMotRev 9  // black
#define RightMotFwd 12 // brown
#define RightMotRev 5 // red
#define LeftSpeedPin 11 // grey
#define RightSpeedPin 4 // orange

int const INTERRUPT_PIN = 13;  // Define the interruption #13 pin
int leftEncoderPin1 = 6; // yellow
int leftEncoderPin2 = 7; // green
int rightEncoderPin1 = 2; // orange
int rightEncoderPin2 = 3; // red

volatile int lastLeftEncoded = 0;
volatile long leftEncoderValue = 0;
volatile int lastRightEncoded = 0;
volatile long rightEncoderValue = 0;

// CONSTANTS
double const IN_PER_REV = 7.5;
double const CM_PER_REV = IN_PER_REV * 2.54;
double const DRIVE_EPSILON = 0.01;
double const ROTATION_EPSILON = 0.01;

float euler[3]

void setup() {
  // MOTOR INITIALIZE ============================
  pinMode(LeftMotFwd, OUTPUT);
  pinMode(LeftMotRev, OUTPUT);
  pinMode(RightMotFwd, OUTPUT);
  pinMode(RightMotRev, OUTPUT);
  pinMode(LeftSpeedPin, OUTPUT);
  pinMode(RightSpeedPin, OUTPUT);

  Serial.begin(9600);

  pinMode(leftEncoderPin1, INPUT_PULLUP);
  pinMode(leftEncoderPin2, INPUT_PULLUP);
  pinMode(rightEncoderPin1, INPUT_PULLUP);
  pinMode(rightEncoderPin2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPin1), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin2), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin1), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin2), updateRightEncoder, CHANGE);

  // DMP INITIALIZE ===============================
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if (mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  } else {
    Serial.println("MPU6050 connection successful");
  }

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
}

void loop() {
  analogWrite(LeftSpeedPin, 150); // Adjust left motor speed from 0-255
  analogWrite(RightSpeedPin, 150); // Adjust right motor speed from 0-255

  // RUN COMMANDS: =======================

  driveDistanceCM(10)
  delay(1000)
  rotateDegrees()
  driveDuration(3000)

  // =====================================

  delay(100);
}

//* Rotate where rotation is in degrees and *//
void rotateDegrees(double deg, bool turnCW) { // TODO: fix direction of turning i.e. CW or CCW cuz idk what dir it will actually go in; also assuming that euler is in deg
  mpu.dmpGetEuler(euler, &q);
  digitalWrite(LeftMotFwd,  turnCW ? HIGH : LOW )
  digitalWrite(LeftMotRev,  turnCW ? LOW  : HIGH)
  digitalWrite(RightMotFwd, turnCW ? LOW  : HIGH)
  digitalWrite(RightMotRev, turnCW ? HIGH : LOW )

  if (abs(euler[0] - deg) > ROTATION_EPSILON) { // I think [0] is pitch
    delay(200)
  }

  resetDrive()
}

//* Drive for a duration where input is in seconds and direction to drive in *//
void driveDuration(bool driveForwards, double seconds) {
  digitalWrite(LeftMotFwd,  driveForwards ? HIGH : LOW )
  digitalWrite(LeftMotRev,  driveForwards ? LOW  : HIGH)
  digitalWrite(RightMotFwd, driveForwards ? HIGH : LOW )
  digitalWrite(RightMotRev, driveForwards ? LOW  : HIGH)
  delay(seconds * 1000)

  resetDrive()
}

//* Drive to a distance where input is in cm (negative distance will reverse) *//
void driveDistanceCM(double distance) {
  double revolutions = distance / CM_PER_REV;
  bool dir = distance < 0

  digitalWrite(LeftMotFwd,  dir ? LOW  : HIGH)
  digitalWrite(LeftMotRev,  dir ? HIGH : LOW )
  digitalWrite(RightMotFwd, dir ? LOW  : HIGH)
  digitalWrite(RightMotRev, dir ? HIGH : LOW )

  while (abs(revolutions - leftEncoderValue) > DRIVE_EPSILON) { // TODO: Fix ??? idk if left encoder already knows its distane but it shouldn;t because that depends on wheel size ?? (?)
    delay(200)
  }

  resetDrive()
}

//* Set motor drive to 0 *//
void resetDrive() {
  digitalWrite(LeftMotFwd,  LOW)
  digitalWrite(LeftMotRev,  LOW)
  digitalWrite(RightMotFwd, LOW)
  digitalWrite(RightMotRev, LOW)
}

//* BROKEN ENCODER CODE MAYBE *//
void updateLeftEncoder() {
  int MSB = digitalRead(leftEncoderPin1);
  int LSB = digitalRead(leftEncoderPin2);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastLeftEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) leftEncoderValue--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) leftEncoderValue++;

  lastLeftEncoded = encoded;
}

void updateRightEncoder() {
  int MSB = digitalRead(rightEncoderPin1);
  int LSB = digitalRead(rightEncoderPin2);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastRightEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) rightEncoderValue--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) rightEncoderValue++;

  lastRightEncoded = encoded;
}