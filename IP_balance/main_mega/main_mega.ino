/* NOTE: this program is meant to be the main loop on the arduino mega,
 *       meaning that it will operate the motors input from computer, output to motors
 *       and it will input encoder readings and send back to the computer
 * - The IMU and servo control will be left up for the computer itself for now
 *   as of April 17, 2020
 * =======
 * 
 * THE EXPECTED SIZE OF ARRAY SHOULD BE 11
 */

// TODO: tune the PID gains
// NOTE: right should be negative to be the same as left (left goes forward with positive)

#include <string.h>
#include <stdio.h>
#include <limits.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PID_v1.h"

// Using a simple PID
#define LOG_INPUT 1
#define MANUAL_TUNING 1
#define LOG_PID_CONSTANTS 0 // MANUAL_TUNING must be 1 for this
#define MOVE_BACK_AND_FORTH 0
#define MIN_ABS_SPD 25

// Reset Pin, if HIGH then arduino mega  will reset
#define RESET_PIN 7

// Right Motor  Pins
#define INA_1 3
#define INB_1 4
#define PWM_1 5

// Left Motor Pins
#define INA_2 12
#define INB_2 13
#define PWM_2 11

// Left Encoder
#define Left_Encoder_PinA 18
#define Left_Encoder_PinB 19

volatile long Left_Encoder_Ticks = 0;
// Variable to read current state of left encoder pin
volatile bool LeftEncoderBSet;

// Right Encoder
#define Right_Encoder_PinA 16
#define Right_Encoder_PinB 17

volatile long Right_Encoder_Ticks = 0;
// Variable to read current state of right encoder pin
volatile bool RightEncoderBSet;

// Var for time updating
unsigned long lastUpdateMicrosecs = 0;
unsigned long lastUpdateMillisecs = 0;
unsigned long currentMicrosecs = 0;
unsigned long microsecsSinceLastUpdate = 0;
float secsSinceLastUpdate = 0;

// Motor speed from mega
int motorLspd = 0;
int motorRspd = 0;
int _currentSpeed = 0;

// Motor speed inputs from the pc over serial
int motorLspd_RX = 0;
int motorRspd_RX = 0;
bool motorOverride = false;

// MPU
MPU6050 mpu;
// control & status variables for mpu
bool dmpReady = false;  // if DMP init success -> true
uint8_t mpuIntStatus;   // holds actual interrupt status byte from the MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes in current FIFO 
// orientation & motion variables
Quaternion q;           // []                 quaternion
VectorFloat gravity;    // [x, y, z]          gravity vector
float yaw_pitch_roll[3]; // [yaw, pitch, roll] yaw, pitch, roll container & gravity vector

// PID
#if MANUAL_TUNING
double kp , ki, kd;
double prevKp, prevKi, prevKd;
#endif
double originalSetpoint = 174.29;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.3;
double input, output;
int moveState = 0; //0 = balance; 1 = back; 2 = forth

#if MANUAL_TUNING
  PID pid(&input, &output, &setpoint, 0, 0, 0, DIRECT);
#else
  PID pid(&input, &output, &setpoint, 70, 240, 1.9, DIRECT);
#endif

// timers
long time1Hz = 0;
long time5Hz = 0;


// ====== MPU FUNCTIONS ======
volatile bool mpuInterrupt = false; // indicate if MPU interrupt pin is high
void dmpDataReady(){
  mpuInterrupt = true;
}

// ===== CONTROL FUNCTIONS =====
void loopAt1Hz(){
  #if MANUAL_TUNING
  setPIDTuningValues();
  #endif
}

void loopAt5Hz(){
  #if MOVE_BACK_AND_FORTH
  moveBackForth();
  #endif
}

void moveBackForth(){
  moveState++;
  if (moveState > 2) moveState = 0;
  if (moveState == 0)
    setpoint = originalSetpoint;
    
  else if (moveState == 1)
    setpoint = originalSetpoint - movingAngleOffset;
  else
    setpoint = originalSetpoint + movingAngleOffset;
}

// PID tuning (use 3 potentiometers)
#if MANUAL_TUNING
void setPIDTuningValues()
{
  readPIDTuningValues();
  if (kp != prevKp || ki != prevKi || kd != prevKd){
#if LOG_PID_CONSTANTS
      Serial.print(kp);Serial.print(", ");Serial.print(ki);Serial.print(", ");Serial.println(kd);
#endif
      pid.SetTunings(kp, ki, kd);
      prevKp = kp; prevKi = ki; prevKd = kd;
    }
}


void readPIDTuningValues()
{
  int potKp = analogRead(A8);
  int potKi = analogRead(A9);
  int potKd = analogRead(A10);
  kp = map(potKp, 0, 1023, 0, 25000) / 100.0; //0 - 250
  ki = map(potKi, 0, 1023, 0, 100000) / 100.0; //0 - 1000
  kd = map(potKd, 0, 1023, 0, 500) / 100.0; //0 - 5
}
#endif

//====== SERIAL FUNCTIONS ======
// Read in input from pi
void readSerialInput(){
  if (Serial.available() > 0){
    // NOTE: incoming data should be in form: "l+###r-###" w/ + & - interchangeable
    String data = Serial.readStringUntil('\n');
    // parse through string to get motor speed updates
    char data_char[data.length() + 1];
    strcpy(data_char, data.c_str()); // use strcpy() to copy the c-string into a char array
    const char delim[2] = ";";
    char *l_token, *r_token;
    l_token = strtok(data_char, delim); // get first token of the string data
    r_token = strtok(NULL, delim);

    // update motor speeds
    motorLspd_RX = interpretData(l_token);
    motorRspd_RX = interpretData(r_token);
    motorOverride = true;
  }
  else{
    //Serial.println("ERROR: serial was not available...\n");
  }
}

//interpret the msg for the speed
int interpretData(char *input_token){ // NOTE: *input_token is equiv. to input_token[0]
  int spd, sign;
  if(input_token[1] == '+'){
    spd = charArrayToInt(input_token);
  }
  else if(input_token[1] == '-'){
    sign = -1;
    spd = charArrayToInt(input_token) * sign;
  }
  return spd;
}

//convert char array in int, NOTE: default length of token should be 5
int charArrayToInt(char *input_token){
  int result;
  result = (input_token[2] - '0') * 100 + (input_token[3] - '0') * 10 + (input_token[4] - '0');
  return result;
}


//====== TIME FUNCTIONS ======
void updateTime(){
  currentMicrosecs = micros();
  lastUpdateMillisecs = millis();
  microsecsSinceLastUpdate = currentMicrosecs - lastUpdateMillisecs;

  if (microsecsSinceLastUpdate < 0){
    microsecsSinceLastUpdate = INT_MIN - lastUpdateMicrosecs + currentMicrosecs;
  }

  lastUpdateMicrosecs = currentMicrosecs;
  secsSinceLastUpdate = microsecsSinceLastUpdate / 1000000.0;

  Serial.print("t");
  Serial.print(";");
  Serial.print(lastUpdateMicrosecs);
  Serial.print(";");
  Serial.print(secsSinceLastUpdate);
  Serial.print("\n");
}


//====== ENCODER FUNCTIONS ======
void setupEncoders(){
  // Quadrature encoders
  // Left encoder
  pinMode(Left_Encoder_PinA, INPUT);      // sets pin A pullup
  pinMode(Left_Encoder_PinB, INPUT);      // sets pin B pullup
  attachInterrupt(digitalPinToInterrupt(Left_Encoder_PinA), do_Left_Encoder, RISING);

  // Right encoder
  pinMode(Right_Encoder_PinA, INPUT);      // sets pin A pullup
  pinMode(Right_Encoder_PinB, INPUT);      // sets pin B pullup

  attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinA), do_Right_Encoder, RISING);
}

void updateEncoders(){
  Serial.print("e");
  Serial.print(";");
  Serial.print(Left_Encoder_Ticks);
  Serial.print(";");
  Serial.print(Right_Encoder_Ticks);
  Serial.print("\n");
}

void do_Left_Encoder(){
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  Left_Encoder_Ticks += LeftEncoderBSet ? -1 : +1;
}

void do_Right_Encoder(){
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;
}


//====== MOTOR FUNCTIONS ======
void setupMotors(){
  // Right Motor
  pinMode(INA_1, OUTPUT);
  pinMode(INB_1, OUTPUT);
  pinMode(PWM_1, OUTPUT);

  // Left Motor
  pinMode(INA_2, OUTPUT);
  pinMode(INB_2, OUTPUT);
  pinMode(PWM_2, OUTPUT);
}

// NOTE: move right and left servo functions should be the same code body as the equiv function in
//       motor_encoders_test
void moveRMotor(float spd){
  // NOTE: right should be negative to be the same as left (left goes forward with positive)
  spd = -1 * spd; 
  if (spd > 0){
    analogWrite(PWM_1, spd);
    digitalWrite(INA_1, HIGH);
    digitalWrite(INB_1, LOW);
  }
  else if(spd < 0){
    analogWrite(PWM_1, abs(spd));
    digitalWrite(INA_1, LOW);
    digitalWrite(INB_1, HIGH);
  }
  else{ // stop
    //analogWrite(PWM_1, spd);
    digitalWrite(INA_1, LOW);
    digitalWrite(INB_1, LOW);
  }
}

void moveLMotor(float spd){
  if (spd > 0){
    analogWrite(PWM_2, spd);
    digitalWrite(INA_2, LOW);
    digitalWrite(INB_2, HIGH);
  }
  else if(spd < 0){
    analogWrite(PWM_2, abs(spd));
    digitalWrite(INA_2, HIGH);
    digitalWrite(INB_2, LOW);
  }
  else{ // stop
    //analogWrite(PWM_1, spd);
    digitalWrite(INA_2, LOW);
    digitalWrite(INB_2, LOW);
  }
}

void updateMotors(int mls, int mrs){
  moveRMotor(mls);
  moveLMotor(mrs);
}

void updateMotorsFwrd(int mls, int mrs){
  moveRMotor(mls);
  moveLMotor(mrs);
}

void moveMotors(int speed, int minAbsSpeed){
  int direction = 1;
  if (speed < 0){
    direction = -1;
    speed = min(speed, direction * MIN_ABS_SPD);
    speed = max(speed, -255);
  }
  else{
    speed = max(speed, MIN_ABS_SPD);
    speed = min(speed, 255);
  }

  if (speed == _currentSpeed) return;

  int realSpeed = max(MIN_ABS_SPD, abs(speed));

  updateMotorsFwrd(realSpeed, realSpeed);

  _currentSpeed = direction * realSpeed;
}

void updateMotorsPi(int mls, int mrs){
  Serial.print("s");
  Serial.print(";");
  Serial.print(mls);
  Serial.print(";");
  Serial.print(mrs);
  Serial.print("\n");
}


//======= RESET PIN FUNCTIONS =======
void setupResetPin(){
  digitalWrite(RESET_PIN, HIGH);
  delay(200);
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
}

void resetBoard(){
  Serial.println();
  delay(1000);
  digitalWrite(RESET_PIN, LOW);
}







//====== SETUP FUNCTION ======
void setup(){
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	// Init Serial port with 115200 buad rate
  Serial.begin(115200);
  //init I2C devices
  mpu.initialize();
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0){
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);  
  }
  else{
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

	setupEncoders();
  setupMotors();
  setupResetPin();
}


//====== MAIN LOOP ======
// NOTE: cannot call any delay()'s in the main loop due to long delays causing
//       overflow on the MPU6050's buffer, leading to inability to read from it
void loop(){
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // wait for MPU interrupt or extra packet(s) available

  
  // TODO: mpuInterrupt seems to be a bit broken here, stuck in this loop?
  
  
  while (!mpuInterrupt && fifoCount < packetSize){
    // no mpu data - perform PID calculations and output to motors
    pid.Compute();
    // ===========================================================
    // TODO: MOVE MOTORS OUTPUT WITH SLOW SPEED
    // ===========================================================
    //Serial.print("PID Output u: "); Serial.println(output);
    moveMotors(output, MIN_ABS_SPD);


    unsigned long currentMillis = millis();

    if (currentMillis - time1Hz >= 1000){
      loopAt1Hz();
      time1Hz = currentMillis;
    }
    if (currentMillis - time5Hz >= 5000){
      loopAt5Hz();
      time5Hz = currentMillis;
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024){
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02){
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(yaw_pitch_roll, &q, &gravity);
      #if LOG_INPUT
          Serial.print("yaw_pitch_roll\t");
          Serial.print(yaw_pitch_roll[0] * 180/M_PI);
          Serial.print("\t");
          Serial.print(yaw_pitch_roll[1] * 180/M_PI);
          Serial.print("\t");
          Serial.println(yaw_pitch_roll[2] * 180/M_PI);
      #endif
      input = yaw_pitch_roll[1] * 180/M_PI + 180;
  }

  readSerialInput(); // update mega of motor spd from pi
  updateTime();      // update time to pi
  updateEncoders();  // update encoders to pi
  if (motorOverride){
    updateMotors(motorLspd_RX, motorRspd_RX);
    motorOverride = false;
  }
  else{
    updateMotorsPi(motorLspd, motorRspd); // update motor spd to pi
    updateMotors(motorLspd, motorRspd);
  }
}
