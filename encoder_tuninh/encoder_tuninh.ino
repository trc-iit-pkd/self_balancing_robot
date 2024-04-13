#include <ESP32Encoder.h>

ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

// motor-left
const int motorPinDirectionLeft = 18;  
const int motorPinSpeedLeft = 19;  
const int encoderPinA_Left = 4;  
const int encoderPinB_Left = 16;

//motor-right
const int motorPinDirectionRight = 13;  
const int motorPinSpeedRight = 14;  
const int encoderPinA_Right = 2;  
const int encoderPinB_Right = 15;
 
volatile int i = 0;

// Globals
float deltaTLeft;
long currTLeft;
long prevTLeft = 0;

float deltaTRight;
long currTRight;
long prevTRight = 0;

volatile int posLeft = 0;
volatile int posPrevLeft = 0;
volatile int posRight = 0;
volatile int posPrevRight = 0;

float vel_rpm_Left = 0;
float vel_rpm_Right = 0;

float u_Left = 0;
float u_Right = 0;

int pwr_Left = 0;
int pwr_Right = 0;

// PID params
float errPrev_Left = 0;
float err_Left;
float errPrev_Right = 0;
float err_Right;

// target speed in rpm
float vtLeft = 20;
float vtRight = 20; 

// PID gains
float kp_Left = 9.0;
float ki_Left = 500.0;
float kd_Left = 0.0;

float kp_Right = 9.0;
float ki_Right = 500.0;
float kd_Right = 0.0;

// PID integral terms
float eintegral_Left = 0;
float eintegral_Right = 0;

// filter params
float v1Filt_Left = 0;
float v1Prev_Left = 0;
float v1Filt_Right = 0;
float v1Prev_Right = 0;

volatile float velocity_Left = 0;
volatile float velocity_Right = 0;

void setup() {
  Serial.begin(115200);  
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  encoderLeft.attachFullQuad(encoderPinA_Left, encoderPinB_Left);  // Left motor
  encoderRight.attachFullQuad(encoderPinA_Right, encoderPinB_Right); // Right Motor
    
  encoderLeft.setCount(0);
  encoderRight.setCount(0);

  Serial.println("Encoder Left Start = " + String((int32_t)encoderLeft.getCount()));
  Serial.println("Encoder Right Start = " + String((int32_t)encoderRight.getCount()));

  pinMode(motorPinDirectionLeft, OUTPUT);
  pinMode(motorPinSpeedLeft, OUTPUT);
  pinMode(motorPinDirectionRight, OUTPUT);
  pinMode(motorPinSpeedRight, OUTPUT);
}

void loop() {
  currTLeft = micros();
  posLeft = encoderLeft.getCount(); 
  deltaTLeft = ((float) (currTLeft - prevTLeft)) / 1.0e6;
  velocity_Left = (posLeft - posPrevLeft) / deltaTLeft;
  posPrevLeft = posLeft;
  prevTLeft = currTLeft;

  currTRight = micros();
  posRight = encoderRight.getCount();
  deltaTRight = ((float) (currTRight - prevTRight)) / 1.0e6;
  velocity_Right = (posRight - posPrevRight) / deltaTRight;
  posPrevRight = posRight;
  prevTRight = currTRight;
 
  vel_rpm_Left = (velocity_Left * 60)/(34560); // GearRatio=540:1, CPR=32
  vel_rpm_Right = (velocity_Right * 60)/(17280); // GearRatio=540:1, CPR=16
  
  v1Filt_Left = lowPassFilter(vel_rpm_Left, v1Prev_Left);
  v1Filt_Right = lowPassFilter(vel_rpm_Right, v1Prev_Right); // 25HZ cuttoff
  
  u_Left = pidControl(vtLeft, v1Filt_Left, kp_Left, ki_Left, kd_Left, err_Left, errPrev_Left, eintegral_Left, deltaTLeft);
  u_Right = pidControl(vtRight, v1Filt_Right, kp_Right, ki_Right, kd_Right, err_Right, errPrev_Right, eintegral_Right, deltaTRight);
  
  if(u_Right >= 30.0 || u_Left >= 30.0){
    u_Right = 30.0;
    u_Left = 30.0;
  }
  else if(u_Right <= -30.0 || u_Left <= -30.0){
    u_Right = -30.0;
    u_Left = -30.0;
  }

  setMotor(u_Left, motorPinDirectionLeft, motorPinSpeedLeft);
  setMotor(u_Right, motorPinDirectionRight, motorPinSpeedRight);

  // Serial.print("dtLeft: ");
  // Serial.print(deltaTLeft);
  // Serial.print(", "),
  // Serial.print("enLeftCounts: ");
  // Serial.print(posLeft);
  // Serial.print(", ");
  // Serial.print("vel_left: ");
  // Serial.print(velocity_Left);
  // Serial.print(", ");
  // Serial.print("vel_rpm_Left");
  // Serial.print(vel_rpm_Left);
  // Serial.print(", ");


  // Serial.print("dtRight: ");
  // Serial.print(deltaTRight);
  // Serial.print(", "),
  // Serial.print("enRightCounts: ");
  // Serial.print(posRight);
  // Serial.print(", ");
  // Serial.print("vel_right: ");
  // Serial.print(velocity_Right);
  // Serial.print(", ");
  // Serial.print("vel_rpm_Right: ");
  // Serial.print(vel_rpm_Right);
  // Serial.println();

  Serial.print("targ_vel:");
  Serial.print(vtRight);
  Serial.print(",");
  Serial.print("vel_right:");
  Serial.print(u_Right);
  Serial.print(",");
  Serial.print("vel_left:");
  Serial.print(u_Left);
  // Serial.println(",");
  // Serial.print("vel_left:");
  // Serial.print(v1Filt_Left);
  // Serial.println(",");
  // Serial.print("vel_left:");
  // Serial.print(v1Filt_Right);
  Serial.println();
  delay(50);
}

float pidControl(float vTar_, float vAct_, float kp_, float ki_, float kd_, float &err_, float &errPrev_, float &eintegral_, float dt_) {
  err_ = vTar_ - vAct_;
  eintegral_ += (dt_ / 1.0e6) * (err_);
  float uControl = kp_ * err_ + ki_ * eintegral_ + ((kd_ * (err_ - errPrev_)/(dt_)))*1.0e6;
  errPrev_ = err_;
  return uControl;
}

float lowPassFilter(float v1_, float &v1Prev_) {
  float v1Filt_ = 0.854 * v1Filt_Left + 0.0728 * v1_ + 0.0728 * v1Prev_;
  v1Prev_ = v1_;
  return v1Filt_;
}

void setMotor(float uControl, int in1, int in2) {
  
  int dir = (uControl >= 0) ? 1 : -1;

  // MAPPING pwm->rpm
  int pwmVal = map(abs(int(uControl)), 0, 30, 0, 255);
  analogWrite(in2, pwmVal); // Motor speed
  digitalWrite(in1, (dir == 1) ? HIGH : LOW);
  // Serial.print("pwm:");
  // Serial.print(pwmVal);
  // Serial.println();
}
