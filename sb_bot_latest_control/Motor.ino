void initializeMotor(){
  // Initialize pins
  pinMode(AENABLE, OUTPUT);
  pinMode(APHASE, OUTPUT);
  pinMode(BENABLE, OUTPUT);
  pinMode(BPHASE, OUTPUT);
}

void moveMotor(int motorSpeed){
  int dir = (motorSpeed > 0) ? 1 : 0;
  // Write motor speed
  if(motorSpeed<=0){
    motorSpeed = 255-abs(motorSpeed);
  }
  analogWrite(AENABLE, (abs(motorSpeed) + motorAoffset));
  analogWrite(BENABLE, (abs(motorSpeed) + motorBoffset));
  if(dir ==1){
  digitalWrite(APHASE,LOW);
  digitalWrite(AENABLE, HIGH);

  digitalWrite(BPHASE,LOW);
  digitalWrite(BENABLE, HIGH);
}
else{
  digitalWrite(APHASE,HIGH);
  digitalWrite(AENABLE, LOW);

  digitalWrite(BPHASE,HIGH);
  digitalWrite(BENABLE, LOW); 
}
}


void stopMotor(){
  digitalWrite(APHASE,LOW);
  digitalWrite(AENABLE, LOW);

  digitalWrite(BPHASE,LOW);
  digitalWrite(BENABLE, LOW); 
}

void calculateWay(){
  encoderAposition = EncoderA.getCount();
  encoderBposition = EncoderB.getCount();
  way1 = (encoderAposition * 2 * M_PI * 0.035 / 540 / 32);
  way2 = (encoderBposition * 2 * M_PI * 0.035 / 540 / 16);
  way = (way1+way2)/2; // Calculate distance: 0.035 = radius of the wheel in meters; 540 = gear ratio; 32 = number of counts per revolution of the magnetic disc
  // Serial.println(way);
}
