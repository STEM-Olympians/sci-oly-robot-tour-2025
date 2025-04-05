#include <Wire.h>
#include <MPU6050.h>
#include <PinChangeInterrupt.h>


#define LeftMotFwd 9  // white
#define LeftMotRev 10  // black
#define RightMotFwd 12 // brown
#define RightMotRev 5 // red
#define LeftSpeedPin 11 // grey
#define RightSpeedPin 4 // orange

int leftEncoderPin1 = 6; // yellow
int leftEncoderPin2 = 7; // green
int rightEncoderPin1 = 2; // orange
int rightEncoderPin2 = 3; // red

volatile int lastLeftEncoded = 0;
volatile long leftEncoderValue = 0;
volatile int lastRightEncoded = 0;
volatile long rightEncoderValue = 0;

void setup() {
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

}

void loop() {
  analogWrite(LeftSpeedPin, 150); // Adjust left motor speed from 0-255
  analogWrite(RightSpeedPin, 150); // Adjust right motor speed from 0-255


  for (int i = 0; i <= 500; i++) {
    digitalWrite(LeftMotFwd, HIGH);
    digitalWrite(LeftMotRev, LOW);
    digitalWrite(RightMotFwd, HIGH);
    digitalWrite(RightMotRev, LOW);
    Serial.print("Forward - Left: ");
    Serial.print(leftEncoderValue);
    Serial.print(" Right: ");
    Serial.println(rightEncoderValue);
  }

  delay(1000);

  for (int i = 0; i <= 500; i++) {
    digitalWrite(LeftMotFwd, LOW);
    digitalWrite(LeftMotRev, HIGH);
    digitalWrite(RightMotFwd, LOW);
    digitalWrite(RightMotRev, HIGH);
    Serial.print("Reverse - Left: ");
    Serial.print(leftEncoderValue);
    Serial.print(" Right: ");
    Serial.println(rightEncoderValue);
  }

  delay(1000);
}

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
