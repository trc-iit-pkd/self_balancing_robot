![image](https://github.com/user-attachments/assets/1837def7-1b37-495c-a296-e1b2fe821bb3)# ============= Prototype-1 ================== 
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
- ![Fusion 360 Icon]([https://example.com/fusion360_icon.png](https://fullycrack.org/wp-content/uploads/2023/03/unnamed.png)) Designed on Autodesk Fusion 360
- Model was mechanical analyzed properly before the compoents were bought and the design was made in the CAD software
- Note: The imu should not be placed so far below the COM of the system. Also make a rought estimate of the natural frequency of the system as it will help in setting the control loop(super loop) frequency for a decent performance.

## Control Algorithm for Self-Balancing
### Hardware
- -->Link to code for PID-control based self-balancing [https://github.com/trc-2023-2024/self_balancing_robot/blob/hardware/sb_bot_control.ino]

## Circuit Connections
![self_balancing circuit](https://github.com/trc-2023-2024/self_balancing_robot/assets/97225407/47423409-0cb3-4e24-9442-a9674bf0c0a8)


## References
- [https://www.instructables.com/Arduino-Self-Balancing-Robot-1/]
# ==========================================================================================
# Prototype-2 Segway motion ---> Need State Space Controllers (work progress on HOLD as of now)
