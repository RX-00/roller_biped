/* NOTE: this program is meant to be the main loop on the arduino mega,
 *       meaning that it will operate the motors input from computer, output to motors
 *       and it will input encoder readings and send back to the computer
 * - The IMU and servo control will be left up for the computer itself for now
 *   as of April 17, 2020
 * =======
 * 
 * THE EXPECTED SIZE OF ARRAY SHOULD BE 11
 */

 // TODO: get IMU working on this and get independent IP balancer up

#include <string.h>
#include <stdio.h>
#include <limits.h>

// Reset Pin, if HIGH then arduino mega  will reset
#define RESET_PIN 7

// Right Motor  Pins
#define INA_1 2
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

// Motor speed inputs from the pc over serial
int motorLspd_RX = 0;
int motorRspd_RX = 0;
bool motorOverride = false;


//====== SETUP FUNCTION ======
void setup(){
	// Init Serial port with 115200 buad rate
  Serial.begin(115200);
	setupEncoders();
  setupMotors();
  setupResetPin();
}


//====== MAIN LOOP ======
void loop(){
  readSerialInput(); // update mega of motor spd from pi
  updateTime();      // update time to pi
  updateEncoders();  // update encoders to pi
  if (motorOverride){
    updateMotors(motorLspd_RX, motorRspd_RX);
    delay(990); // delay 990 ms
    motorOverride = false;
  }
  else{
    updateMotorsPi(motorLspd, motorRspd); // update motor spd to pi
    updateMotors(motorLspd, motorRspd);
  }
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
