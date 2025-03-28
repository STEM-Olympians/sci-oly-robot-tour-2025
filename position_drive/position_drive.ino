
// IMU setup
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

int const INTERRUPT_PIN = 13;  // Define the interruption #13 pin
bool blinkState;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;   // Set true if DMP init was successful
uint8_t MPUIntStatus;    // Holds actual interrupt status byte from MPU
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];  // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;         // [w, x, y, z]         Quaternion container
VectorInt16 aa;       // [x, y, z]            Accel sensor measurements
VectorInt16 gy;       // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            Gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*-Packet structure for InvenSense teapot demo-*/
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;  // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

// Motor setups
#define LeftMotFwd 10    // white
#define LeftMotRev 9     // black
#define RightMotFwd 12   // brown
#define RightMotRev 5    // red
#define LeftSpeedPin 11  // grey
#define RightSpeedPin 4  // orange

int leftEncoderPin1 = 6;   // yellow
int leftEncoderPin2 = 7;   // green
int rightEncoderPin1 = 2;  // orange
int rightEncoderPin2 = 3;  // red

volatile int lastLeftEncoded = 0;
volatile long leftEncoderValue = 0;
volatile int lastRightEncoded = 0;
volatile long rightEncoderValue = 0;



// FIXME: Check if the setup is < 3 sec, because robot has gotta start moving 3 sec after we hit start

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment on this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);  //115200 is required for Teapot Demo output

  while (!Serial)
    ;

  /*Initialize device*/
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


  // FIXME: find the actual offsets lmao

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));  //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();  //Get expected DMP packet size for later comparison
  } else {
    Serial.print(F("DMP Initialization failed (code "));  //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
  pinMode(LED_BUILTIN, OUTPUT);


  // Setup motor pins
  pinMode(LeftMotFwd, OUTPUT);
  pinMode(LeftMotRev, OUTPUT);
  pinMode(RightMotFwd, OUTPUT);
  pinMode(RightMotRev, OUTPUT);
  pinMode(LeftSpeedPin, OUTPUT);
  pinMode(RightSpeedPin, OUTPUT);

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
  if (!DMPReady) return;  // Stop the program if DMP programming fails.

  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {  // Get the Latest packet

    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    //FIXME: not really a fixme, but I assume this one is yaw,
    // we want this one for robot angle
    // See below for float gyroAngle
    Serial.print(ypr[0] * 180 / M_PI);


    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);

    float gyroAngle = ypr[0] * 180 / M_PI;

    float leftDistance = getDistanceCentimeters(leftEncoderValue);
    float rightDistance = getDistanceCentimeters(rightEncoderValue);

    // FIXME: Insert driving logic here


    /* Blink LED to indicate activity */
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
  }
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


// sSpeed must be between 0-255
// The best way to do this, is to find a consistent power, that we know will go a certain distance for a certain time
// Then, we can just stop for the rest of the time
// That way, we can be pretty accurate with our movements
void driveSpeed(double speed) {
  if (speed == 0) {
    stop();
    return;
  }

  if (speed > 0) {
    digitalWrite(LeftMotFwd, HIGH);
    digitalWrite(LeftMotRev, LOW);
    digitalWrite(RightMotFwd, HIGH);
    digitalWrite(RightMotRev, LOW);
  } else {
    digitalWrite(LeftMotFwd, LOW);
    digitalWrite(LeftMotRev, HIGH);
    digitalWrite(RightMotFwd, LOW);
    digitalWrite(RightMotRev, HIGH);
  }
  analogWrite(LeftSpeedPin, speed);
  analogWrite(RightSpeedPin, speed);
}


// Stop the motor
void stop() {
  digitalWrite(LeftMotFwd, LOW);
  digitalWrite(LeftMotRev, LOW);
  digitalWrite(RightMotFwd, LOW);
  digitalWrite(RightMotRev, LOW);
  analogWrite(LeftSpeedPin, 0);
  analogWrite(RightSpeedPin, 0);
}



double getDistanceCentimeters(long encoderValue) {

  // FIXME: Find the real number of ticks in one revolution
  double revolutions = encoderValue / 5026;

  // Distance is revolutions * circumference (pi * diameter (inches)) * 2.54 cm/inches
  return revolutions * (PI * 2.375) * 2.54;
}

// Reset encoder count
void resetLeftEncoder() {
  leftEncoderValue = 0;
}

// Reset encoder count
void resetRightEncoder() {
  leftEncoderValue = 0;
}

void resetBothEncoders() {
  resetLeftEncoder();
  resetRightEncoder();
}

// TODO:
// #1: Figure out how you want to store the directions
// #2: Figure out how to drive a certain distance, and calculate the time from that
// #3: Figure out how to rest for a certain duration to fit the time requirements
// #4: Figure out how to rotate to a certain angle (probably just 90 degrees), while staying in place
// #5: Fix the timings for the angles as well


// All of these other things were stuff I started working on, but then realized you probably have your own ideas on how to do them
// So, I shall leave them here if you want inspiration, but they might be somewhat incomplete
// Also, your code makes an appearance at the end lol, because I was in the middle of porting it

//     void setDistanceTarget(double targetCenti, double time){
//       resetBothEncoders();
//       targetCentimeters = targetCenti;
//       targetTimeMillis = time;
//     }

//     void setAngleTarget(double targetAng, double time){
//       targetAngle = targetAng;
//       targetTimeMillis = time;
//     }

//     /** This will also reset your position for more accurate tracking*/
//     void setTargetCentimeters(double targetCenti){
//       this->resetEncoder();
//       targetCentimeters = targetCenti;
//     }

//     double centiError(){
//         return (this -> getDistanceCentimeters()) - currentPosition;
//     }

//     bool reachedTargetCentimeter(){
//         double currentPosition = getDistanceCentimeters();
//         return abs(targetCentimeters - currentPosition) < TARGET_CENTIMETER_EPSILON
//     }

//     void reachedTargetAngle(){
//         // Implement gyro reading


//         // double currentAngle =
//         // return abs(targetCentimeters - currentPosition) < TARGET_CENTIMETER_EPSILON
//     }

//     /** Speed is -1 to 1*/
//     void driveToTarget() {

//       if(this -> reachedTargetCentimeter()) {
//         this -> drive(0);
//         return;
//       }

//       double currentPosition = getDistanceCentimeters();


//       // Get difference between current position and target position
//       // double frontLeftPower = DriveSubsystem.CalculateDirection(subsystem.getMotorPosition() - target, speed);
//       // double frontRightPower = frontLeftPower;

//       // if (frontLeftPower == 0) {
//       //   // isFinished = true;
//       //   subsystem.resetMotorPosition();
//       // }

//       // // Set motor powers
//       // subsystem.drive(frontRightPower, frontLeftPower);
//   }
// }