// Test with encoders and motors
// NOTE: as of April 12, 2020 the encoders seem fine but the motor commands seem to be incorrect
//       compared to the truth table


// NOTE: TODO YOU NEED TO CHECK EACH THREE SET
//       OF MOTOR PINS AGAIN, CHECK ONE MOTOR SET
//       AT A TIME!!
// testing out the right motor (1)
// TODO: also figure out which way is forward and backward for each motor (CW and CCW)
//       from the PWM signals


// Right Motor  Pins
#define INA_1 PC_6
#define INB_1 PC_5
#define PWM_1 PC_4

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


//====== SETUP FUNCTION ======
void setup(){
	// Init Serial port with 115200 buad rate
  Serial.begin(115200);
	setupEncoders();
  setupMotors();
}


//====== MAIN LOOP ======
void loop(){
  updateEncoders();
  moveRightMotorFor(-25, 2000);
  stopFor(50);
  moveLeftMotorFor(25, 2000);
  stopFor(50);
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

// turn right motor at given speed (+ and - for direction)
void moveRightMotorFor(float spd, int time){ // time in ms
  if (spd > 0){
    analogWrite(PWM_1, spd);
    digitalWrite(INA_1, HIGH);
    digitalWrite(INB_1, LOW);
  }
  else if(spd < 0){
    Serial.println("backwards right");
    analogWrite(PWM_1, abs(spd));
    digitalWrite(INA_1, LOW);
    digitalWrite(INB_1, HIGH);
  }
  else{ // stop
    //analogWrite(PWM_1, spd);
    digitalWrite(INA_1, LOW);
    digitalWrite(INB_1, LOW);
  }
  delay(time);
}

// turn left motor at given speed (+ and - for direction)
void moveLeftMotorFor(float spd, int time){ // time in ms
  if (spd > 0){
    analogWrite(PWM_2, spd);
    digitalWrite(INA_2, LOW);
    digitalWrite(INB_2, HIGH);
  }
  else if(spd < 0){
    Serial.println("backwards left");
    analogWrite(PWM_2, abs(spd));
    digitalWrite(INA_2, HIGH);
    digitalWrite(INB_2, LOW);
  }
  else{ // stop
    //analogWrite(PWM_1, spd);
    digitalWrite(INA_2, LOW);
    digitalWrite(INB_2, LOW);
  }
  delay(time);
}

void stopFor(int time){
  digitalWrite(INA_1, LOW);
  digitalWrite(INB_1, LOW);
  digitalWrite(INA_2, LOW);
  digitalWrite(INB_2, LOW);
  delay(time);
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
