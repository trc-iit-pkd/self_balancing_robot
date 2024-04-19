// Include Libraries
#include "Wire.h"
#include <PID_v1.h>
#include <ESP32Encoder.h>
#include <BluetoothSerial.h>
#include "CytronMotorDriver.h"

BluetoothSerial SerialBT;

ESP32Encoder EncoderA;
ESP32Encoder EncoderB;

// Pin Definitions
//Left motor
#define AENABLE 19   // PWM signal for motor speed
#define APHASE 18    // HIGH, LOW determines motor direction
//Right Motor
#define BENABLE 14   // PWM signal for motor speed
#define BPHASE 13    // HIGH, LOW determines motor direction

//Left Encoder pins
const int ENCODERA1 = 4;
const int ENCODERA2 = 16;
//Right Encoder pins
const int ENCODERB1 = 2;
const int ENCODERB2 = 12;

const uint8_t scl = 22; 
const uint8_t sda = 21; 


// ***Motors***

CytronMD motor1(PWM_PWM, AENABLE, APHASE);  
CytronMD motor2(PWM_PWM, BENABLE, BPHASE);

double motorPWM = 0;  // Motor speed, control signal of the pitch angle PID controller
int motorDirection;

int motorAoffset = 0;
int motorBoffset = 0;

int encoderAposition; // Encoder position of Motor A
int encoderBposition; // Encoder position of Motor B
double way; // Distance the Segway has traveled
double way1; // Distance the Segway has traveled
double way2; // Distance the Segway has traveled

// Offset values from IMU sensor calibration:
double RateRoll, RatePitch, RateYaw;
double RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
double RateCalibrationAR, RateCalibrationAP;
int RateCalibrationNumber;
double AccX, AccY, AccZ;
double AngleRoll, AnglePitch;

double gyDot; // Variables to store the speed around the Y-axis

// kalman stuff
double KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
double KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2, ka_cal=0.0;
double Kalman1DOutput[]={0,0};

double pitchAngleOffset = 0; // Offset of the IMU measurement to the real angle

// ***PID Controllers***
int sampleTime = 10; // How often the PID algorithm is evaluated (in ms)

// Angle PID
int angleControlMin = -255, angleControlMax = 255; // Limits of the output control signal (motorPWM)
double angleControlSetPoint = 0; // Setpoint pitch angle
double angleControlKP = 10, angleControlKI = 200, angleControlKD = 0.1; // PID controller tuning parameters
PID angleControl(&KalmanAnglePitch, &motorPWM, &angleControlSetPoint, angleControlKP, angleControlKI , angleControlKD, DIRECT); // Create the PID controller object; '&' symbol attaches variable to the object

// Way PID
int wayControlMin = -2, wayControlMax = 2; // Limits of the output control signal (motorPWM)
double wayControlSetPoint = 0.0; // Setpoint for path
double wayControlKP = 10, wayControlKI = 0, wayControlKD = 0.1; // PID controller tuning parameters
PID wayControl(&way, &angleControlSetPoint, &wayControlSetPoint, wayControlKP, wayControlKI, wayControlKD, DIRECT); // Create the PID controller object; wayControlSetPoint is the controlled variable and input variable of the angle PID controller

// ***System State***
bool enabled = true; // Specifies whether motors are turned on or off0
bool wayControlEnabled = true; // Specifies whether path control is turned on or off

// ***Timing***
double currentTime = 0; // Variable to store the current time
long lastTime = 0; // Variables to store the time of the last serial output
double wheelTime = 0; // Time since the wheel encoder value measurement
double timerWheel = 0; //Time for wheel encoder
long gyroDt = 0; // Time interval


void setup() {
  Wire.begin(); // Initialize I2C
  Serial.begin(57600); // baud rates affects the rate of data recieved by the arduino/esp32/esp82 not the control algo
   Wire.setClock(400000); // 400kHz (fast mode I2C communications) 
  // PS: this mode for I2C is more than enough for mpu6050 which has a 1kHz data transfer freq.(approx. I think)
  Wire.begin(sda, scl);
  delay(250);
  Wire.beginTransmission(0x68); // reset imu
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  EncoderA.attachFullQuad(ENCODERA2, ENCODERA1);  // Left motor
  EncoderB.attachFullQuad(ENCODERB2, ENCODERB1); // Right Motor
    
  EncoderA.setCount(0);
  EncoderB.setCount(0);

  Serial.println("Encoder Left Start = " + String((int32_t)EncoderA.getCount()));
  Serial.println("Encoder Right Start = " + String((int32_t)EncoderB.getCount()));

  SerialBT.begin("Self Balancing");
  initializePID(); // Initialize PID controllers
  // initializeMotor(); // Initialize motors
  initializeIMU(); // Initialize IMU
  lastTime = millis(); // Start timer
}

void loop() {
  // currentTime = millis();
  calculateAngle(); // Determine pitch angle from IMU
  calculateWay(); // Calculate distance from encoders
  if(wayControlEnabled == true){
    wayControl.Compute(); // Calculate control signal
  }
//   if (abs(angleControlSetPoint) >=3){
//    angleControlSetPoint = 3;
// }
  angleControl.Compute(); // Calculate control signal
  if(enabled == true){
    moveMotor(motorPWM); // Move motors
  }
  printInfo();
  receiveBluetoothData(); 
  // updateKp();
  // updateKi();
  // updateKd();
  // checkSerial();
}
