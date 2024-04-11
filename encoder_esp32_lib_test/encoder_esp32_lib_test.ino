#include <ESP32Encoder.h>

ESP32Encoder encoder;
ESP32Encoder encoder2;

// timer and flag for example, not needed for encoders

const uint8_t right_motor_speed = 14; // D14
const uint8_t right_motor_dir = 13;   // D13

const uint8_t left_motor_speed = 19; // D19
const uint8_t left_motor_dir = 18;   // D18

void setup(){
	
	Serial.begin(115200);
	// Enable the weak pull down resistors

	//ESP32Encoder::useInternalWeakPullResistors = puType::down;
	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors = puType::up;

	// use pin 4 and 16 for the first encoder
	encoder.attachFullQuad(4, 16);  // Left motor
	// use pin 2 and 15 for the second encoder
	encoder2.attachFullQuad(2, 15); // Right Motor
		
	// set starting count value after attaching
	encoder.setCount(0);

	// clear the encoder's raw count and set the tracked count to zero
	encoder2.setCount(0);
	Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));
  Serial.println("Encoder Start = " + String((int32_t)encoder2.getCount()));
	// set the lastToggle
  pinMode(right_motor_speed, OUTPUT);
  pinMode(right_motor_dir, OUTPUT);
  pinMode(left_motor_speed, OUTPUT);
  pinMode(left_motor_dir, OUTPUT);


}

void loop(){
	// Loop and read the count
   Serial.println("Encoder count = " + String((int32_t)encoder.getCount()) + " " + String((int32_t)encoder2.getCount()));
   delay(100);

	// unsigned long startTime = millis();
  //   while (millis() - startTime < 5000)
  //   {
  //       // Run right motor clockwise
  //       digitalWrite(right_motor_dir, HIGH); // Set direction
  //       analogWrite(right_motor_speed, 100); // Set speed (full speed)

  //       // Run left motor counter-clockwise
  //       digitalWrite(left_motor_dir, HIGH);      // Set direction
  //       analogWrite(left_motor_speed, 100); // Set speed (full speed)
  //   }

  //   // Stop motors after 5 seconds
  //   digitalWrite(right_motor_speed, LOW); // Stop right motor
  //   digitalWrite(left_motor_speed, LOW);  // Stop left motor
}

