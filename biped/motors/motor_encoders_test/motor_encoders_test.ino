// Test with encoders and motors
// NOTE: as of April 12, 2020 the encoders seem fine but the motor commands seem to be incorrect
//       compared to the truth table

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

  // NOTE: TODO YOU NEED TO CHECK EACH THREE SET
  //       OF MOTOR PINS AGAIN, CHECK ONE MOTOR SET
  //       AT A TIME!!
  // testing out the right motor (1)

  analogWrite(PWM_1, 20); // input PWM signal
  analogWrite(PWM_2, 20);
  digitalWrite(INA_1, HIGH);
  digitalWrite(INB_1, LOW);
  //digitalWrite(INA_1, HIGH);   // input direction signals (refer to truth table)
  //digitalWrite(INB_1, HIGH);
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
