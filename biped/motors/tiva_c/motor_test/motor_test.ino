// Motor test to make sure they work and figure out left & right compared to motor 1 & 2
// NOTE: these functions only move CW right and CCW left
///Right Motor  Pins
#define INA_1 PA_3
#define INB_1 PA_4
#define PWM_1 PC_6

///Left Motor Pins
#define INA_2 PE_4
#define INB_2 PE_5
#define PWM_2 PC_5

void setup() {
  //Setting Right Motor pin as OUTPUT
  pinMode(INA_1, OUTPUT);
  pinMode(INB_1, OUTPUT);

  //Setting Left Motor pin as OUTPUT
  pinMode(INA_2, OUTPUT);
  pinMode(INB_2, OUTPUT);
}

void loop() {
  // Move one wheel for 3 seconds
  //move_forward();
  delay(3000);
  // Stop for 3 seconds
  stop();
  delay(3000);
}

void move_forward() {
  //Setting CW rotation to Right Motor and CCW to Left Motor
  //Right Motor

  digitalWrite(INA_1, HIGH);
  digitalWrite(INB_1, LOW);
  analogWrite(PWM_1, 255);

  //Left Motor
  digitalWrite(INA_2, LOW);
  digitalWrite(INB_2, HIGH);
  analogWrite(PWM_2, 255);
}

void move_right() {
  //Right Motor
  digitalWrite(INA_1, HIGH);
  digitalWrite(INB_1, LOW);
  analogWrite(PWM_1, 255);

  //Left Motor
  digitalWrite(INA_2, HIGH);
  digitalWrite(INB_2, HIGH);
  analogWrite(PWM_2, 0);
}

void move_left() {
  //Right Motor
  digitalWrite(INA_1, HIGH);
  digitalWrite(INB_1, HIGH);
  analogWrite(PWM_1, 0);

  //Left Motor
  digitalWrite(INA_2, LOW);
  digitalWrite(INB_2, HIGH);
  analogWrite(PWM_2, 255);
}

void stop() {
  //Right Motor
  digitalWrite(INA_1, HIGH);
  digitalWrite(INB_1, HIGH);
  analogWrite(PWM_1, 0);

  //Left Motor
  digitalWrite(INA_2, HIGH);
  digitalWrite(INB_2, HIGH);
  analogWrite(PWM_2, 0);
}

void move_backward() {
  //Right Motor
  digitalWrite(INA_1, LOW);
  digitalWrite(INB_1, HIGH);
  analogWrite(PWM_1, 255);

  //Left Motor
  digitalWrite(INA_2, HIGH);
  digitalWrite(INB_2, LOW);
  analogWrite(PWM_2, 255);
}
