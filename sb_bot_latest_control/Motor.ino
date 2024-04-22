// void initializeMotor(){
//   // Initialize pins
//   // pinMode(AENABLE, OUTPUT);
//   // pinMode(APHASE, OUTPUT);
//   // pinMode(BENABLE, OUTPUT);
//   // pinMode(BPHASE, OUTPUT);

  
// }

void moveMotor(int motorSpeed){

  motor1.setSpeed(motorSpeed);   // Motor 1 runs forward at 50% speed.
  motor2.setSpeed(motorSpeed); 

//   int dir = (motorSpeed > 0) ? 1 : 0;
//   // Write motor speed
//   // if(motorSpeed<=0){
//   //   motorSpeed = 255-abs(motorSpeed);
//   // }
//   analogWrite(AENABLE, ((motorSpeed) + motorAoffset));
//   analogWrite(BENABLE, ((motorSpeed) + motorBoffset));
//   if(dir ==1){
//   digitalWrite(APHASE,LOW);
//   digitalWrite(AENABLE, HIGH);

//   digitalWrite(BPHASE,LOW);
//   digitalWrite(BENABLE, HIGH);
// }
// else  if(dir == 0){
//   digitalWrite(APHASE,HIGH);
//   digitalWrite(AENABLE, LOW);

//   digitalWrite(BPHASE,HIGH);
//   digitalWrite(BENABLE, LOW); 
// }
}


void stopMotor(){
  digitalWrite(APHASE,LOW);
  digitalWrite(AENABLE, LOW);

  digitalWrite(BPHASE,LOW);
  digitalWrite(BENABLE, LOW); 
}

void calculateWay(){
  // currentTime = micros();
  encoderAposition = EncoderA.getCount();
  encoderBposition = EncoderB.getCount();
  way1 = (encoderAposition * 2 * M_PI * 0.035 / 540 / 32);
  way2 = (encoderBposition * 2 * M_PI * 0.035 / 540 / 16);
  // timerWheel = (currentTime - wheelTime)/1.0e6;
  way = (way1+way2)/(2); // Calculate distance: 0.035 = radius of the wheel in meters; 540 = gear ratio; 32 = number of counts per revolution of the magnetic disc
  // Serial.println(encoderAposition);
  // Serial.println(encoderBposition);
  // Serial.println(timerWheel,5);
  Serial.println(way);
  // wheelTime = currentTime;
}
