void printInfo(){
  // Print value on the serial monitor every 50 milliseconds
  if(millis() - lastTime >= 100){
    lastTime = millis();
    //Serial.println(angleControlSetPoint);
    Serial.print("Pitch Angle:");
    Serial.print(KalmanAnglePitch);
    Serial.print(",");
    Serial.print("Roll Angle:");
    Serial.print(KalmanAngleRoll);
    Serial.print(",");
    Serial.print("Motor PWM: ");
    Serial.print(motorPWM);
    Serial.print(",");
    Serial.print("New Setpoint: ");
    Serial.print(angleControlSetPoint);
    Serial.println("");
    // Serial.print('\t');
    // Serial.print(pitchAngle);
    // Serial.println("°");
  }
}

void receiveBluetoothData() {
  if (SerialBT.available()) {
    String dataReceived = SerialBT.readStringUntil('>');
    
    // Find positions of colons
    int firstColon = dataReceived.indexOf(':');
    int secondColon = dataReceived.indexOf(':', firstColon + 1);
    
    // Extract values between colons
    String value1 = dataReceived.substring(1, firstColon);
    String value2 = dataReceived.substring(firstColon + 1, secondColon);
    String value3 = dataReceived.substring(secondColon + 1);

    // Convert strings to integers
    int intValue1 = value1.toDouble();
    int intValue2 = value2.toDouble();
    int intValue3 = value3.toDouble();

    // Serial.print("Recieved value of Kp: ");
    // Serial.println(intValue1);
    // Serial.print("Recieved value of Ki: ");
    // Serial.println(intValue2);
    // Serial.print("Recieved value of Kd: ");
    // Serial.println(intValue3);
    // wayControlSetPoint = intValue3;
    angleControl.SetTunings(intValue1,intValue2,intValue3); // Set new control parameters
    
  }
}


// void checkSerial(){
//   receiveWithStartEndmarker();
//   if(newData == true){
//     strcpy(tempChars, receivedChars); // Copy strings
//     parseData();
//     processParsedData();
//     newData = false;
//   }
// }

// void receiveWithStartEndmarker(){
//   static bool receiveInProgress = false;
//   static byte ndx = 0; // Position in the array where operations are performed; Static: Variable persists after the function ends
//   char startMarker = '<'; // Start character at the beginning of each string
//   char endMarker = '>'; // Terminator character at the end of each string
//   char rc; // Received character
//   while(Serial.available() && newData == false){
//     rc = Serial.read();
//     if(receiveInProgress == true){
//       if(rc != endMarker){
//         receivedChars[ndx] = rc;
//         ndx++;
//         if(ndx >= numChars){
//           ndx = numChars - 1;
//         }
//       }else{
//         receivedChars[ndx] = '\0';
//         ndx = 0;
//         newData = true;
//         receiveInProgress = false;
//       }
//     }else if(rc == startMarker){
//         receiveInProgress = true;
//     }
//   }
// }

// void parseData(){ // Split the data into its components
//   char * strtokIndex; // Pointer
//   strtokIndex = strtok(tempChars, ","); // Convert strings to tokens
//   strcpy(message, strtokIndex);

//   strtokIndex = strtok(NULL, ",");
//   floatMessage = atof(strtokIndex);
// }

// void processParsedData(){
//   if(strcmp(message,"wKP") == 0){ // strcmp compares two strings
//     Serial.print("Changing KP value to: ");
//     Serial.println(floatMessage);
//     angleControlKP = floatMessage;
//     angleControl.SetTunings(angleControlKP,angleControlKI,angleControlKD); // Set new control parameters
//   }
//   if(strcmp(message,"wKI") == 0){ // strcmp compares two strings
//     Serial.print("Changing KI value to: ");
//     Serial.println(floatMessage);
//     angleControlKI = floatMessage;
//     angleControl.SetTunings(angleControlKP,angleControlKI,angleControlKD); // Set new control parameters
//   }
//   if(strcmp(message,"wKD") == 0){ // strcmp compares two strings
//     Serial.print("Changing KD value to: ");
//     Serial.println(floatMessage);
//     angleControlKD = floatMessage;
//     angleControl.SetTunings(angleControlKP,angleControlKI,angleControlKD); // Set new control parameters
//   }

//   // Turn motors on and off
//   if(strcmp(message,"en") == 0 && floatMessage == 1){ // strcmp compares two strings
//     Serial.println("Turning motors on...");
//     enabled = true;
//   }
//   if(strcmp(message,"en") == 0 && floatMessage == 0){ // strcmp compares two strings
//     Serial.println("Turning motors off...");
//     enabled = false;
//     stopMotor();
//   }
// }
