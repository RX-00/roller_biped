#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PID_v1.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 20

MPU6050 mpu;

#define DEBUG
#ifdef DEBUG
//#define DPRINT(args...)  Serial.print(args)             //OR use the following syntax:
#define DPRINTSTIMER(t)    for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define  DPRINTSFN(StrSize,Name,...) {char S[StrSize];Serial.print("\t");Serial.print(Name);Serial.print(" "); Serial.print(dtostrf((float)__VA_ARGS__ ,S));}//StringSize,Name,Variable,Spaces,Percision
#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#else
#define DPRINTSTIMER(t)    if(false)
#define DPRINTSFN(...)     //blank line
#define DPRINTLN(...)      //blank line
#endif

#define LED_PIN 13

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


//PID
double originalSetpoint = 170;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
int moveState=0; //0 = balance; 1 = back; 2 = forth
double Kp = 15;
double Kd = 0;
double Ki = 0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.99;
double motorSpeedFactorRight = 0.25;

//timers
long time1Hz = 0;
long time5Hz = 0;



// You may use MPU6050_calibration.ino (https://github.com/Protonerd/DIYino/blob/master/MPU6050_calibration.ino)
// to find the offest values for your MPU6050. 
// If you require high precision, you may wish to fine tune it by printing out some parameters.
//                       XA      YA      ZA      XG      YG      ZG
int MPUOffsets[6] = {  2885,  3523,    1919,     77,     235,     106};


// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has changed
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void MPU6050Connect() {
  static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize();
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; //only try 10 times
    MPU6050Connect(); // Lets try again
    return;
  }
  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  Serial.print("mpu.getInterruptDrive=  "); Serial.println(mpu.getInterruptDrive());
  attachInterrupt(0, dmpDataReady, CHANGE); // pin 2 on the Uno. Please check the online Arduino reference for more options for connecting this interrupt pin
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  mpu.resetFIFO(); // Clear fifo buffer
  mpuInterrupt = false; // wait for next interrupt
}

// ================================================================
// ===                    MPU DMP Get Data                      ===
// ================================================================
void readMPUFIFOBuffer() {
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  
  // Check for overflow
  if((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    
  // check for incorrect packet size
  }else if((!fifoCount) || (fifoCount % packetSize)){ // something's wrong. reset and try again.
    Serial.println(F("Wrong packet size!"));
    digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();// clear the buffer and start over
    
  // otherwise, check for DMP data ready interrupt (this should happen almost always)
  }else if(mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)){
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // discard the rest of the values(packets) inside buffer to prevent overflow. We don't need every single packet MPU6050 gives us.
    mpu.resetFIFO();


    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the LED to indicate activity
  }else{
    // This usually never happens.
  }
}


// ================================================================
// ===                    Output Functions                      ===
// ================================================================
// add these functions to your code as needed

// get quaternion components in a [w, x, y, z] format
// very useful when Euler and YPR angles cannot satisfy your application
// refer to https://en.wikipedia.org/wiki/Quaternion for more information
void getQuaternion()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
}
void printQuaternion()
{
  // display quaternion values in easy matrix form: w x y z
  Serial.print("quat\t");
  Serial.print(q.w);
  Serial.print("\t");
  Serial.print(q.x);
  Serial.print("\t");
  Serial.print(q.y);
  Serial.print("\t");
  Serial.println(q.z);
}

// The Euler angles are in radians. Divide it by M_PI then multiply it by 180 to get angles in degrees.
// Note that Euler angles suffer from gimbal lock (for more info, see http://en.wikipedia.org/wiki/Gimbal_lock)
// Try calculating your parameters from quaternions if you experience gimbal lock.
void getEuler()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
}
void printEuler()
{
  // display Euler angles in degrees
  Serial.print("euler\t");
  Serial.print(euler[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(euler[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(euler[2] * 180/M_PI);
}

// Yaw, pitch, and roll angles are in radians. Divide it by M_PI then multiply it by 180 to get angles in degrees.
// Note that these angles suffer from gimbal lock (for more info, see http://en.wikipedia.org/wiki/Gimbal_lock)
// Try calculating your parameters from quaternions if you experience gimbal lock.
void getYawPitchRoll()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}
void printYawPitchRoll()
{
  // display yaw, pitch, roll in degrees
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);
}

// Use this if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, use getWorldAccel() instead.
void getRealAccel()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
}
void printRealAccel()
{
  Serial.print("areal\t");
  Serial.print(aaReal.x);
  Serial.print("\t");
  Serial.print(aaReal.y);
  Serial.print("\t");
  Serial.println(aaReal.z);
}

// Use this if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
void getWorldAccel()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
}
void printWorldAccel()
{
  Serial.print("aworld\t");
  Serial.print(aaWorld.x);
  Serial.print("\t");
  Serial.print(aaWorld.y);
  Serial.print("\t");
  Serial.println(aaWorld.z);
}



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
/*  currentMicrosecs = micros();
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
  Serial.print("\n");*/
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
  moveRMotor(mls * motorSpeedFactorRight);
  moveLMotor(mrs * motorSpeedFactorLeft);
}

void moveMotors(int speed, int minAbsSpeed){
  int direction = 1;
  if (speed < 0){
    direction = -1;
    speed = min(speed, direction * MIN_ABS_SPEED);
    speed = max(speed, -255);
  }
  else{
    speed = max(speed, MIN_ABS_SPEED);
    speed = min(speed, 255);
  }

  if (speed == _currentSpeed) return;

  int realSpeed = max(MIN_ABS_SPEED, abs(speed));

  if (speed > 0) updateMotorsFwrd(-realSpeed, -realSpeed);
  if (speed < 0) updateMotorsFwrd(realSpeed, realSpeed);

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






// ================================================================
// ===                         Setup                            ===
// ================================================================
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // setup IMU MPU6050
  Serial.begin(115200); //115200
  while (!Serial);
  Serial.println("i2cSetup");
  i2cSetup();
  Serial.println("MPU6050Connect");
  MPU6050Connect();

  // setup PID
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);

  // setup encoders, motors, etc.
  Serial.println("Setup encoders");
	setupEncoders();
  Serial.println("Setup motors");
  setupMotors();
  Serial.println("Setup reset pin");
  //setupResetPin();

  Serial.println("Setup complete");
  pinMode(LED_PIN, OUTPUT);
}
// ================================================================
// ===                          Loop                            ===
// ================================================================
void loop() {
  // ---- read from the FIFO buffer ---- //
  // The interrupt pin could have changed for some time already. 
  // So set mpuInterrupt to false and wait for the next interrupt pin CHANGE signal.
  mpuInterrupt = false; 
  //  Wait until the next interrupt signal. This ensures the buffer is read right after the signal change.
  while(!mpuInterrupt){
    pid.Compute();
    Serial.println(output);
    //motorController.move(output, MIN_ABS_SPEED);
    moveMotors(output, MIN_ABS_SPEED); // NOTE: might need a delay after this line...
  }
  delay(5);
  mpuInterrupt = false;
  readMPUFIFOBuffer();

  getYawPitchRoll();

  /*
  Serial.print("ypr (degrees)\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);
  */

  input = ypr[1] * 180/M_PI + 180;


  //printYawPitchRoll();
}
