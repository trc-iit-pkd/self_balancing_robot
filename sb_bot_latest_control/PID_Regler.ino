void initializePID(){
  angleControl.SetMode(AUTOMATIC); // Set PID controller to automatic mode
  angleControl.SetOutputLimits(angleControlMin,angleControlMax); // Set limits of the output control signal (motorPWM)
  angleControl.SetSampleTime(sampleTime);
  wayControl.SetMode(AUTOMATIC); // Set PID controller to automatic mode
  wayControl.SetOutputLimits(wayControlMin,wayControlMax); // Set limits of the output control signal (motorPWM)
  wayControl.SetSampleTime(sampleTime);
}
