# ============= Prototype-1 ================== 
# Self-Balancing Robot 
## Required Components:
- ESP-8266(Nodemcu-82)
- Motor Driver MDD3A (PWM-PWM motor driver)
- Orange Planetary Gear DC Motor 12V 359RPM 34.5 N-cm PG36M555-13.7K
- MPU9050 Imu
- DC-DC Buck Converter (24v/12v to 5v/5A)
- 11.1V 3S LiPo Battery (2200mAh 30C)
- OE-37 Hall Effect Magnetic Encoders(useful for the next phase of the system for state space based segway motion control)

## Robot Design
- Designed on Autodesk Fusion 360.
- The model was mechanically analyzed properly before the components were bought and the design was made in the CAD software.
- **Note**: The IMU should not be placed too far below the COM (Center of Mass) of the system. Also, make a rough estimate of the natural frequency of the system as it will help in setting the control loop (super loop) frequency for better performance.
- <img src="https://fullycrack.org/wp-content/uploads/2023/03/unnamed.png" alt="Fusion 360 Icon" width="50" height="50" style="vertical-align:middle;"> 

## Control Algorithm for Self-Balancing
### Installations
- For Arduino IDE (https://www.arduino.cc/en/software)
- For The ESP82-Arduino Core, put the link beside[https://arduino.esp8266.com/stable/package_esp8266com_index.json] in the URL manager in Arduino Preferences section.
### Hardware
The control algorithm is primarily based on feedback from the IMU (MPU9050/6050). The core logic involves driving the motors in the direction of the robot’s tilt, allowing the motor torque to counteract the imbalance and prevent the robot from tipping over. Since the system is underactuated and inherently marginally unstable, the control strategy ensures continuous motor adjustments to maintain balance.

- A simple PID controller is used to achieve this stabilization.
- The code was written in the Arduino IDE (C programming language) using the ESP82-Arduino Core.
- Feedback from the IMU consists mainly of accelerometer and gyroscope data, specifically roll, pitch, roll rate, and pitch rate. The gyroscope data is integrated to derive roll and pitch rates, which are then fused with accelerometer data to eliminate noise and errors in individual streams, providing fully filtered roll and pitch values.
**Note**: Set the I2C communication to fast mode by configuring the I2C frequency to 400kHz.

- The filtered roll and pitch values are then sent to the PID controller, which calculates the control output (PWM signals) to adjust motor behavior and balance the robot.
**Note**: For improved performance, constrain the control velocity within maximum and minimum PWM values, considering the motor’s operating voltage and the input voltage from power source.

- Link to code for PID-control based self-balancing --> [https://github.com/trc-2023-2024/self_balancing_robot/blob/hardware/sb_bot_control.ino]
<img src="https://static-00.iconduck.com/assets.00/arduino-ide-icon-2048x2025-x4ims8sb.png" alt="Arduino IDE Icon" width="50" height="50" style="vertical-align:middle;">
### Hardware Results
- <img src="https://drive.google.com/drive/u/0/folders/1tUwDEiyPsDD87bBJ_FoU0sQTpsBuYyro" alt="Self Balancing Robot" width="480" height="640" style="vertical-align:middle;"> 
- link to video here ---> https://youtu.be/jyWlr8lFBLY
  
## Circuit Connections
![self_balancing circuit](https://github.com/trc-2023-2024/self_balancing_robot/assets/97225407/47423409-0cb3-4e24-9442-a9674bf0c0a8)

## References
- [https://www.instructables.com/Arduino-Self-Balancing-Robot-1/]
 ==========================================================================================
# Prototype-2 Segway motion ---> Need State Space Controllers (work progress on HOLD as of now)
