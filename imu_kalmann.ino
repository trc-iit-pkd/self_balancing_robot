#include <Wire.h>
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

struct PID{
  double kp = 1.0;
  double kd = 0.0;
  double ki = 0.0;
}pid;

long int pwm = 0; // (0,255)
long int pwmMax = 255; // maxpwm val

double set_point = 0.0; // angle in degrees
double prevErr = 0.0;
double eintegral = 0.0;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}
void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}
void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();
}
void loop() {
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

  double controlVel = pidControlOutput(set_point, KalmanAnglePitch, pid.kp, pid.kd, pid.ki, prevErr, LoopTimer, eintegral); // apply to body frame long vel of the robot 
  pwm = (int) (controlVel);

  // Serial.print(", Z: ");
  // Serial.print(g.gyro.z);
  // Serial.println(" rad/s");

  Serial.print("pwm_val: ");
  Serial.print(pwm);
  Serial.print(", ");
  Serial.print("Control_input");
  Serial.println(controlVel);
  Serial.print("Roll Angle [°] ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch Angle [°] ");
  Serial.print(KalmanAnglePitch);
  Serial.println("");

  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}

double pidControlOutput(double target_pitch, double current_pitch, double kp_, double kd_, double ki_, double &prevErr_, uint32_t &prevTime_, double &eintegral_){
  uint32_t currentTime = micros();
  double dt = (double) (currentTime-prevTime_)*1.0e-6;
  prevTime_ = currentTime;

  double err = target_pitch-current_pitch;
  eintegral += err*dt;
  prevErr_ = err;

  double controlInput = kp_*err + kd_*(err-prevErr_)/dt + ki_*(eintegral);
  return controlInput;

}


void setMotorSpeed(uint8_t pwm_pin, uint8_t dir_pin, int dir, int pwm_val){
  analogWrite(pwm_pin, pwm_val);
  if(dir == 0){
    digitalWrite(dir_pin, HIGH);
  }
  else if(dir == 1){
    digitalWrite(dir_pin, LOW);
  }
}