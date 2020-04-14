/* NOTE: this program is meant to be the main loop on the tiva c launchpad,
 *       meaning that it will operate the motors input from computer, output to motors
 *       and it will input encoder readings and send back to the computer
 * - The IMU and servo control will be left up for the computer itself for now
 *   as of April 14, 2020
 * =======
 */

#include <Messenger.h>
#include <limits.h>

// Reset Pin, if HIGH then tiva c launchpad will reset
#define RESET_PIN PB_2

// Right Motor  Pins
#define INA_1 PA_3
#define INB_1 PA_4
#define PWM_1 PC_6

// Left Motor Pins
#define INA_2 PE_4
#define INB_2 PE_5
#define PWM_2 PC_5

// Left Encoder
#define Left_Encoder_PinA PF_4
#define Left_Encoder_PinB PD_7

volatile long Left_Encoder_Ticks = 0;
// Variable to read current state of left encoder pin
volatile bool LeftEncoderBSet;

// Right Encoder
#define Right_Encoder_PinA PD_6
#define Right_Encoder_PinB PC_7

volatile long Right_Encoder_Ticks = 0;
// Variable to read current state of right encoder pin
volatile bool RightEncoderBSet;

// Messenger Object
Messenger Messenger_Handler = Messenger();

// Var for time updating
unsigned long lastUpdateMicrosecs = 0;
unsigned long lastUpdateMillisecs = 0;
unsigned long currentMicrosecs = 0;
unsigned long microsecsSinceLastUpdate = 0;
float secsSinceLastUpdate = 0;

// Motor speed inputs from the pc over serial
float motorLspd = 0;
float motorRspd = 0;


//====== SETUP FUNCTION ======
void setup(){
	// Init Serial port with 115200 buad rate
  Serial.begin(115200);
	setupEncoders();
  setupMotors();
  setupResetPin();
  Messenger_Handler.attach(onMsgComplete);
}


//====== MAIN LOOP ======
void loop(){
  readSerialInput();
  updateTime();
  updateEncoders();
  updateMotors();
}



//====== SERIAL FUNCTIONS ======
//Read input from computer over serial port to the tiva c launchpad
void readSerialInput(){
  while(Serial.available() > 0){
    int data = Serial.read();
    Messenger_Handler.process(data);
  }
}

void onMsgComplete(){
  char reset[] = "r";
  char set_spd[] = "s";

  if (Messenger_Handler.checkString(reset)){
    resetBoard();
    Serial.println("Machine resetting has finished...");
  }

  if (Messenger_Handler.checkString(set_spd)){
    setSpd();
    return;
  }
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
  Serial.print("\t");
  Serial.print(lastUpdateMicrosecs);
  Serial.print("\t");
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
  Serial.print("\t");
  Serial.print(Left_Encoder_Ticks);
  Serial.print("\t");
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

void setSpd(){
  motorRspd = Messenger_Handler.readLong();
  motorLspd = Messenger_Handler.readLong();
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

void updateMotors(){
  moveRMotor(motorRspd);
  moveLMotor(motorLspd);

  Serial.print("s");
  Serial.print("\t");
  Serial.print(motorLspd);
  Serial.print("\t");
  Serial.print(motorRspd);
  Serial.print("\n");
}


//======= RESET PIN FUNCTIONS =======
void setupResetPin(){
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);

  // NOTE: You need to physically connect the RESET pins to the ones on the tiva c
  digitalWrite(RESET_PIN, HIGH);
}

void resetBoard(){
  digitalWrite(GREEN_LED, HIGH);
  delay(999);
  digitalWrite(RESET_PIN, LOW);
  digitalWrite(GREEN_LED, LOW);
}
