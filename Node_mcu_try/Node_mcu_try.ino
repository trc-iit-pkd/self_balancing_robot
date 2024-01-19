#include <Wire.h>
#include <math.h>

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication 
const uint8_t scl = 5; // D1
const uint8_t sda = 4; // D2

const uint8_t right_motor_speed = 12; // 
const uint8_t right_motor_dir = 13;
const uint8_t left_motor_speed = 15;
const uint8_t  left_motor_dir = 3; 

struct PID{
  double kp = 1.0;
  double kd = 0.0;
  double ki = 0.0;
}pid;

double cmf_gain = 0.665; // complementary filter gain
double pitch_filt = 0.0;

long int pwm = 50; // (0,255)
long int pwmMax = 255; // maxpwm val

double set_point = 0.0; // angle in degrees
double prevErr = 0.0;
double eintegral = 0.0;

// sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t AccelScaleFactor = 16384; // +-2g full scale
const uint16_t GyroScaleFactor = 131; // += 250 full scale

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

unsigned long currTime, prevTime=0; 
double loopTime;
double gyroAngle=0;
float pitch, roll;

void setup() {
  Serial.begin(115200);
  Wire.begin(sda, scl);
  MPU6050_Init();
}

void loop() {
  currTime = micros();
  loopTime = (double) (currTime - prevTime);
  prevTime = currTime;
  // prevTime2 = (double) (prevTime);
  double Ax, Ay, Az, T, Gx, Gy, Gz;
  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  
  //divide each with their sensitivity scale factor
  Ax = (double)AccelX/AccelScaleFactor*9.81 + 0.1;
  Ay = (double)AccelY/AccelScaleFactor*9.81 + 0.33-0.25;
  Az = (double)AccelZ/AccelScaleFactor*9.81 + 0.51-0.39-1.0;
  // T = (double)Temperature/340+36.53; //temperature formula
  Gx = (double)GyroX/GyroScaleFactor + 0.38 + 1.10;
  Gy = (double)GyroY/GyroScaleFactor - 0.31-0.90;
  Gz = (double)GyroZ/GyroScaleFactor - 0.12-0.33;


  delay(50);
  gyroAngle = gyroAngle + Gy * loopTime/1.0e6;
  getAngle(Ax, Ay, Az); // processes the roll and pitch global variables

  pitch_filt = cmf_gain*(gyroAngle+pitch_filt) + (1-cmf_gain)*pitch;

  double controlVel = pidControlOutput(set_point, pitch_filt, pid.kp, pid.kd, pid.ki, prevErr, prevTime, eintegral); // apply to body frame long vel of the robot 
  pwm = pwm + (int) (controlVel);

  Serial.print("pwm_val: ");
  Serial.print(pwm);
  Serial.print(", ");
  Serial.print("Accelerometer_pitch: ");
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print("gyro_pitch: ");
  Serial.print(gyroAngle);
  Serial.print(", ");
  Serial.print("pitch_filtered: ");
  Serial.print(pitch_filt);
  Serial.print(", ");
  Serial.print("Control_input");
  Serial.println(controlVel);
  Serial.println(loopTime); 
  Serial.println();

  // delay(10);
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


void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init(){
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}

void getAngle(float Vx, float Vy, float Vz) {
  float x = Vx;
  float y = Vy;
  float z = Vz;
  pitch = atan(x / sqrt((y * y) + (z * z)));
  roll = atan(y / sqrt((x * x) + (z * z)));
  //convert radians into degrees
  pitch = pitch * (180.0 / 3.14);
  roll = roll * (180.0 / 3.14) ;
}