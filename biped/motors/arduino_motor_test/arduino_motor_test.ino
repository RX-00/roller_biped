// Test with encoders and motors
// NOTE: level shifters messed up the signal from the tiva c to the motor controller, so now
//       using an arduino mega for simplicity (5V logic with 5V logic)

// Right Motor  Pins
#define INA_1 3 // used to be 2 until MPU6050 implementation
#define INB_1 4
#define PWM_1 5

// Left Motor Pins
#define INA_2 12
#define INB_2 13
#define PWM_2 11

// Left Encoder (2)
#define Left_Encoder_PinA 20
#define Left_Encoder_PinB 21

volatile long Left_Encoder_Ticks = 0;
// Variable to read current state of left encoder pin
volatile bool LeftEncoderBSet;

// Right Encoder (1)
#define Right_Encoder_PinA 18
#define Right_Encoder_PinB 19

volatile long Right_Encoder_Ticks = 0;
// Variable to read current state of right encoder pin
volatile bool RightEncoderBSet;


//====== SETUP FUNCTION ======
void setup() {
  // Init Serial port with 115200 buad rate
  Serial.begin(115200);
  setupEncoders();
  setupMotors();
}


//====== MAIN LOOP ======
void loop() {
  updateEncoders();
  moveRightMotorFor(50, 200);
  moveLeftMotorFor(50, 200);
  moveRightMotorFor(-50, 200);
  moveLeftMotorFor(-50, 200);
}



//====== MOTOR FUNCTIONS ======
void setupMotors() {
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
void moveRightMotorFor(float spd, int time) { // time in ms
  if (spd > 0) {
    analogWrite(PWM_1, spd);
    digitalWrite(INA_1, HIGH);
    digitalWrite(INB_1, LOW);
  }
  else if (spd < 0) {
    analogWrite(PWM_1, abs(spd));
    digitalWrite(INA_1, LOW);
    digitalWrite(INB_1, HIGH);
  }
  else { // stop
    //analogWrite(PWM_1, spd);
    digitalWrite(INA_1, LOW);
    digitalWrite(INB_1, LOW);
  }
  delay(time);
}

// turn left motor at given speed (+ and - for direction)
void moveLeftMotorFor(float spd, int time) { // time in ms
  if (spd > 0) {
    analogWrite(PWM_2, spd);
    digitalWrite(INA_2, LOW);
    digitalWrite(INB_2, HIGH);
  }
  else if (spd < 0) {
    analogWrite(PWM_2, abs(spd));
    digitalWrite(INA_2, HIGH);
    digitalWrite(INB_2, LOW);
  }
  else { // stop
    //analogWrite(PWM_1, spd);
    digitalWrite(INA_2, LOW);
    digitalWrite(INB_2, LOW);
  }
  delay(time);
}

void stopFor(int time) {
  digitalWrite(INA_1, LOW);
  digitalWrite(INB_1, LOW);
  digitalWrite(INA_2, LOW);
  digitalWrite(INB_2, LOW);
  delay(time);
}


//====== ENCODER FUNCTIONS ======
void setupEncoders() {
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

void updateEncoders() {
  Serial.print("e");
  Serial.print("\t");
  Serial.print(Left_Encoder_Ticks);
  Serial.print("\t");
  Serial.print(Right_Encoder_Ticks);
  Serial.print("\n");
}

void do_Left_Encoder() {
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  Left_Encoder_Ticks += LeftEncoderBSet ? -1 : +1;
}

void do_Right_Encoder() {
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;
}
