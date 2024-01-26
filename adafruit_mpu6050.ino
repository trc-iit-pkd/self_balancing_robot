// Basic demo for accelerometer readings from Adafruit MPU6050

// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://RandomNerdTutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

double loopTime;
double gyroAngle=0;
double gyroAngleDeg=0;
unsigned long currTime, prevTime=0;

Adafruit_MPU6050 mpu;

struct PID{
  double kp = 1.0;
  double kd = 0.0;
  double ki = 0.0;
}pid;

double cmf_gain = 0.665; // complementary filter gain
double pitch_filt = 0.0;

long int pwm = 0; // (0,255)
long int pwmMax = 255; // maxpwm val

double set_point = 0.0; // angle in degrees
double prevErr = 0.0;
double eintegral = 0.0;

float pitch, roll;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    // while (1) {
    //   delay(10);
    // }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  currTime = micros();
  loopTime = (double) (currTime - prevTime);
  prevTime = currTime;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  // Serial.print("Acceleration X: ");
  // Serial.print(a.acceleration.x);
  // Serial.print(", Y: ");
  // Serial.print(a.acceleration.y);
  // Serial.print(", Z: ");
  // Serial.print(a.acceleration.z);
  // Serial.println(" m/s^2");

  // Serial.print("Rotation X: ");
  // Serial.print(g.gyro.x);
  // Serial.print(", Y: ");


  gyroAngle = gyroAngle + g.gyro.y * loopTime/1.0e6;
  gyroAngleDeg = (gyroAngle/3.14)*180;
  // Serial.print(gyroAngleDeg);

  getAngle(a.acceleration.x, a.acceleration.y, a.acceleration.z); // processes the roll and pitch global variables
  pitch_filt = cmf_gain*(gyroAngle+pitch_filt) + (1-cmf_gain)*pitch;

  double controlVel = pidControlOutput(set_point, pitch_filt, pid.kp, pid.kd, pid.ki, prevErr, prevTime, eintegral); // apply to body frame long vel of the robot 
  pwm = (int) (controlVel);

  // Serial.print(", Z: ");
  // Serial.print(g.gyro.z);
  // Serial.println(" rad/s");

  Serial.print("pwm_val: ");
  Serial.print(pwm);
  Serial.print(", ");
  Serial.print("Accelerometer_pitch: ");
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print("gyro_pitch: ");
  Serial.print(gyroAngleDeg);
  Serial.print(", ");
  Serial.print("pitch_filtered: ");
  Serial.print(pitch_filt);
  Serial.print(", ");
  Serial.print("Control_input");
  Serial.println(controlVel);
  // Serial.println(loopTime); 
  Serial.println("");
  delay(50);
}

void getAngle(float Vx, float Vy, float Vz) {
  float x = Vx;
  float y = Vy;
  float z = Vz;
  pitch = atan(-x / sqrt((y * y) + (z * z)));
  roll = atan(y / sqrt((x * x) + (z * z)));
  //convert radians into degrees
  pitch = pitch * (180.0 / 3.14);
  roll = roll * (180.0 / 3.14) ;
}

double pidControlOutput(double target_pitch, double current_pitch, double kp_, double kd_, double ki_, double &prevErr_, unsigned long &prevTime_, double &eintegral_){
  unsigned long currentTime = micros();
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