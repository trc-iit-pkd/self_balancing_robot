void printInfo(){
  // Print value on the serial monitor every 50 milliseconds
  if(millis() - lastTime >= 100){
    SerialBT.print(way);
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
    Serial.print(",");
    Serial.print("New Setpoint Way: ");
    Serial.print(wayControlSetPoint);
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
    int thirdColon = dataReceived.indexOf(':', secondColon + 1);
    int fourthColon = dataReceived.indexOf(':', thirdColon + 1);
    int fifthColon = dataReceived.indexOf(':', fourthColon + 1);
    int sixthColon = dataReceived.indexOf(':', fifthColon + 1);

    // Extract values between colons
    String value1 = dataReceived.substring(1, firstColon);
    String value2 = dataReceived.substring(firstColon + 1, secondColon);
    String value3 = dataReceived.substring(secondColon + 1, thirdColon);
    String value4 = dataReceived.substring(thirdColon + 1, fourthColon);
    String value5 = dataReceived.substring(fourthColon + 1, fifthColon);
    String value6 = dataReceived.substring(fifthColon + 1, sixthColon);
    String value7 = dataReceived.substring(sixthColon + 1);

    // Convert strings to integers
    float floatValue1 = value1.toDouble();
    float floatValue2 = value2.toDouble();
    float floatValue3 = value3.toDouble();
    float floatValue4 = value4.toDouble();
    float floatValue5 = value5.toDouble();
    float floatValue6 = value6.toDouble();
    float floatValue7 = value7.toFloat();

    angleControl.SetTunings(floatValue1,floatValue2,floatValue3);
    wayControl.SetTunings(floatValue4,floatValue5,floatValue6);
    wayControlSetPoint = floatValue7;

    // Print the values
    // Serial.print("Value 1: ");
    // Serial.println(floatValue1);
    // Serial.print("Value 2: ");
    // Serial.println(floatValue2);
    // Serial.print("Value 3: ");
    // Serial.println(floatValue3);
    // Serial.print("Value 4: ");
    // Serial.println(floatValue4);
    // Serial.print("Value 5: ");
    // Serial.println(floatValue5);
    // Serial.print("Value 6: ");
    // Serial.println(floatValue6);
    // Serial.print("Value 7: ");
    // Serial.println(floatValue7);
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
