#include <I2Cdev.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

//initialise object
MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
VectorFloat gravity;    // [x, y, z]            gravity vector
Quaternion q;           // [w, x, y, z]         quaternion container
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Motor Control
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define enA 3
#define in1 4
#define in2 5

#define enB 12
#define in3 10
#define in4 11

// For PID Controller (Change these)
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
float Kp = 1;             // (P)roportional Tuning Parameter
float Ki = 2;             // (I)ntegral Tuning Parameter        
float Kd = 3;             // (D)erivative Tuning Parameter
float targetAngle = 0;  // Can be adjusted according to centre of gravity 

float lastpitch;          // Keeps track of error over time
float iTerm;              // Used to accumulate error (integral)

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===              Set up connection to MPU6050                ===
// ================================================================
void MPU6050Connect(){
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-740);
  mpu.setYAccelOffset(-4764);
  mpu.setZAccelOffset(1334);
  mpu.setXGyroOffset(74);
  mpu.setYGyroOffset(-40);
  mpu.setZGyroOffset(87);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    
    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    
    mpu.resetFIFO(); // Clear fifo buffer
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
      if(devStatus == 1) Serial.println("> Initial Memory Load Failed");
      else if (devStatus == 2) Serial.println("> DMP Configuration Updates Failed");
  }
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // really up to you depending on your project)
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  MPU6050Connect();
}


// ================================================================
// ===                       GET IMU DATA                       ===
// ================================================================

void IMUData(){
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      fifoCount = mpu.getFIFOCount();
      Serial.println(F("FIFO overflow!"));
  
  // otherwise, check for DMP data ready interrupt (this should happen @ 100Hz)
  }else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.resetFIFO();
  }
}


// ================================================================
// ===                     PID CONTROLLER                       ===
// ================================================================

int PID(){
  // Calculate pitch
  float pitch = ypr[1] * 180/M_PI;
  
  // Calculate Error
  float error = targetAngle - pitch;

  // Calculate PID terms
  float pTerm = Kp * error;
  iTerm += Ki * error * 10;
  float dTerm = Kd * (pitch - lastpitch) / 10;
  lastpitch = pitch;

  // Obtain PID output value
  float PIDValue = pTerm + iTerm - dTerm;

  // Cap values so be able to send the correct PWM signal to the motors
  if (PIDValue > 255) PIDValue = 255;
  else if (PIDValue < -255) PIDValue = -255;
  
  return int(PIDValue);
}


// ================================================================
// ===                   MOTOR CONTROLLER                       ===
// ================================================================

void MotorDriver(int PIDValue){
    // Forward
  if(PIDValue >= 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, PIDValue);
    
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, PIDValue);
    }
    // Backwards
  else{
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, PIDValue * -1);
    
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, PIDValue * -1);
    }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }
    }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  
  IMUData();

  // Call PID() to send appropriate motor control signal
  MotorDriver(PID());
}
