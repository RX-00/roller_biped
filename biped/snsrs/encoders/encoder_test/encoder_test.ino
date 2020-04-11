// Test code for the encoders, modified from another source
// Encoder pins definition

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


void setup(){
	// Init Serial port with 115200 buad rate
  Serial.begin(9600);  
	SetupEncoders();
}


void SetupEncoders(){
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

void loop(){
	Update_Encoders();
}


void Update_Encoders(){
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
