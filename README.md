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
The control algorithm is primarily based on feedback from the IMU (MPU9050/6050). The core logic is to drive the motors in the direction of the robot’s tilt, enabling the motor torque to counteract the torque caused by the imbalance. This prevents the robot from tipping over. Since the system is underactuated and inherently marginally unstable, the control strategy ensures that the motors continuously adjust to maintain balance.
- For the above purpose a simple PID controller was used to achieve the above.
- The code was written in Arduino IDE (C-programming language) with the ESP82-Arudino Core.
- 

- -->Link to code for PID-control based self-balancing [https://github.com/trc-2023-2024/self_balancing_robot/blob/hardware/sb_bot_control.ino]
<img src="https://static-00.iconduck.com/assets.00/arduino-ide-icon-2048x2025-x4ims8sb.png" alt="Arduino IDE Icon" width="50" height="50" style="vertical-align:middle;">
## Circuit Connections
![self_balancing circuit](https://github.com/trc-2023-2024/self_balancing_robot/assets/97225407/47423409-0cb3-4e24-9442-a9674bf0c0a8)


## References
- [https://www.instructables.com/Arduino-Self-Balancing-Robot-1/]
# ==========================================================================================
# Prototype-2 Segway motion ---> Need State Space Controllers (work progress on HOLD as of now)
