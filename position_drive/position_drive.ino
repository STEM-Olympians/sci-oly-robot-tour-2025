
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_6Axis_MotionApps612.h" // Uncomment this library to work with DMP 6.12 and comment on the above library.

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

/* OUTPUT FORMAT DEFINITION-------------------------------------------------------------------------------------------
- Use "OUTPUT_READABLE_QUATERNION" for quaternion commponents in [w, x, y, z] format. Quaternion does not 
suffer from gimbal lock problems but is harder to parse or process efficiently on a remote host or software 
environment like Processing.

- Use "OUTPUT_READABLE_EULER" for Euler angles (in degrees) output, calculated from the quaternions coming 
from the FIFO. EULER ANGLES SUFFER FROM GIMBAL LOCK PROBLEM.

- Use "OUTPUT_READABLE_YAWPITCHROLL" for yaw/pitch/roll angles (in degrees) calculated from the quaternions
coming from the FIFO. THIS REQUIRES GRAVITY VECTOR CALCULATION.
YAW/PITCH/ROLL ANGLES SUFFER FROM GIMBAL LOCK PROBLEM.

- Use "OUTPUT_READABLE_REALACCEL" for acceleration components with gravity removed. The accel reference frame
is not compensated for orientation. +X will always be +X according to the sensor.

- Use "OUTPUT_READABLE_WORLDACCEL" for acceleration components with gravity removed and adjusted for the world
reference frame. Yaw is relative if there is no magnetometer present.

-  Use "OUTPUT_TEAPOT" for output that matches the InvenSense teapot demo. 
-------------------------------------------------------------------------------------------------------------------------------*/
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_TEAPOT

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

  /*Wait for Serial input*/
  // Serial.println(F("\nSend any character to begin: "));
  // while (Serial.available() && Serial.read()); // Empty buffer
  // while (!Serial.available());                 // Wait for data
  // while (Serial.available() && Serial.read()); // Empty buffer again

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

  // FIXME: Check if the setup is < 3 sec, because robot has gotta start moving 3 sec after we hit start

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
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);

    float gyroAngle = ypr[0] * 180 / M_PI;
#endif

#ifdef OUTPUT_READABLE_QUATERNION
    /* Display Quaternion values in easy matrix form: [w, x, y, z] */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    /* Display real acceleration, adjusted to remove gravity */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    /* Display initial world-frame acceleration, adjusted to remove gravity
      and rotated based on known orientation from Quaternion */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    /* Display quaternion values in InvenSense Teapot demo format */
    teapotPacket[2] = FIFOBuffer[0];
    teapotPacket[3] = FIFOBuffer[1];
    teapotPacket[4] = FIFOBuffer[4];
    teapotPacket[5] = FIFOBuffer[5];
    teapotPacket[6] = FIFOBuffer[8];
    teapotPacket[7] = FIFOBuffer[9];
    teapotPacket[8] = FIFOBuffer[12];
    teapotPacket[9] = FIFOBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++;  // PacketCount, loops at 0xFF on purpose
#endif

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


// Stop the motor
void driveSpeed(double speed) {
  if(speed == 0) {
    stop();
    return;
  } 

  if( speed > 0){
    digitalWrite(LeftMotFwd, HIGH);
  digitalWrite(LeftMotRev, LOW);
  digitalWrite(RightMotFwd, HIGH);
  digitalWrite(RightMotRev, LOW);
  }else {
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


    // Get the encoder count
    long getEncoderCount() {
        return encoderCount;
    }

    double getRevolutions(){
      // Find how many ticks in a rev
        return encoderCount / 2048; // FIXME
    }

    double getDistanceCentimeters(){
      // Distance is revolutions * circumference (pi * diameter (inches)) * 2.54 cm/inches
        return this->getRevolutions() * PI * 2.375  * 2.54; // FIXME
    }
    
    // Reset encoder count
    void resetEncoder() {
        encoderCount = 0;
    }

    void setDistanceTarget(double targetCenti, double time){
      this->resetEncoder();
      targetCentimeters = targetCenti;
      targetTimeMillis = time;
    }

    void setAngleTarget(double targetAng, double time){
      targetAngle = targetAng;
      targetTimeMillis = time;
      
    }



    /** This will also reset your position for more accurate tracking*/
    void setTargetCentimeters(double targetCenti){
      this->resetEncoder();
      targetCentimeters = targetCenti;
    }

    double centiError(){
        return (this -> getDistanceCentimeters()) - currentPosition;
    }

    bool reachedTargetCentimeter(){
        double currentPosition = getDistanceCentimeters();
        return abs(targetCentimeters - currentPosition) < TARGET_CENTIMETER_EPSILON
    }

    void reachedTargetAngle(){
        // Implement gyro reading


        // double currentAngle = 
        // return abs(targetCentimeters - currentPosition) < TARGET_CENTIMETER_EPSILON
    }

    /** Speed is -1 to 1*/
    void driveToTarget() {

      if(this -> reachedTargetCentimeter()) {
        this -> drive(0);
        return;
      }

      double currentPosition = getDistanceCentimeters();

      if(target)


      // Get difference between current position and target position
      // double frontLeftPower = DriveSubsystem.CalculateDirection(subsystem.getMotorPosition() - target, speed); 
      // double frontRightPower = frontLeftPower; 

      // if (frontLeftPower == 0) {
      //   // isFinished = true;
      //   subsystem.resetMotorPosition();
      // }

      // // Set motor powers
      // subsystem.drive(frontRightPower, frontLeftPower);
  }
}

