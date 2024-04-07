
// Analog pins connected to the encoders
const int encoderPinA_Motor1 = 33;  // Connect encoder1 signal A to analog pin A0
const int encoderPinB_Motor1 = 32;  // Connect encoder1 signal B to analog pin A1

volatile int encoderCounts_Motor1 = 0;     // Total counts of encoder1
volatile int encoderDirection_Motor1 = 0;  // Rotation direction of encoder1


void setup() {
  pinMode(encoderPinA_Motor1, INPUT_PULLUP);
  pinMode(encoderPinB_Motor1, INPUT_PULLUP);

  // Attach interrupts to the encoder pins for both motors
  attachInterrupt(digitalPinToInterrupt(encoderPinA_Motor1), updateEncoderMotor1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB_Motor1), updateEncoderMotor1, CHANGE);
  Serial.begin(9600);
}

void loop() {
  // Read and display the encoder counts and direction for both motors
  int currentCounts_Motor1 = encoderCounts_Motor1;
  int currentDirection_Motor1 = encoderDirection_Motor1;


  Serial.println("Encoder Values:");
  Serial.print("Motor 1 - Counts: ");
  Serial.print(currentCounts_Motor1);
  Serial.print(", Direction: ");
  Serial.println(currentDirection_Motor1);

}

void updateEncoderMotor1() {
  static int previousEncoderPinAState_Motor1 = HIGH;

  int currentEncoderPinAState = digitalRead(encoderPinA_Motor1);
  int currentEncoderPinBState = digitalRead(encoderPinB_Motor1);

  if (previousEncoderPinAState_Motor1 == HIGH && currentEncoderPinAState == LOW) {
    if (currentEncoderPinBState == LOW) {
      encoderCounts_Motor1++;
      encoderDirection_Motor1 = 1;  // Clockwise rotation
    } else {
      encoderCounts_Motor1--;
      encoderDirection_Motor1 = -1; // Counter-clockwise rotation
    }
  }

  previousEncoderPinAState_Motor1 = currentEncoderPinAState;
}
