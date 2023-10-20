void postthings(){
   if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    
    // New BME280 sensor readings
    temp = bme.readTemperature();
    //temp = 1.8*bme.readTemperature() + 32;
    
    hum = bme.readHumidity();
    
    pres = bme.readPressure()/100.0F;

    MoistVal1 = analogRead(moist_pin1);  //put Sensor insert into soil
   // Serial.println("Read Value");
   // Serial.println(MoistVal1);
   MoistVal2 = analogRead(moist_pin2);  //put Sensor insert into soil
   MoistVal3 = analogRead(moist_pin3);  //put Sensor insert into soil
   MoistVal4 = analogRead(moist_pin4);  //put Sensor insert into soil
    moistpercent1 = map(MoistVal1, dry, wet, 0, 100);
   // Serial.println("Map Value");
    //Serial.println(moistpercent1);
    moistpercent2 = map(MoistVal2, dry, wet, 0, 100);
    moistpercent3 = map(MoistVal3, dry, wet, 0, 100);
    moistpercent4 = map(MoistVal4, dry, wet, 0, 100);
         
          if(moistpercent1 >= 100){
                moistpercent1 = 100;
                }
          else if(moistpercent1 <=0)
                {
                moistpercent1 = 0;
                }
          else if(moistpercent1 >0 && moistpercent1 < 100)
                {
                moistpercent1 = moistpercent1;
                }

          if(moistpercent2 >= 100){
                moistpercent2 = 100;
                }
          else if(moistpercent2 <=0)
                {
                  moistpercent2 = 0;
                }
          else if(moistpercent2 >0 && moistpercent2 < 100)
                {
                  moistpercent2 = moistpercent2;
                  }

          if(moistpercent3 >= 100){
                moistpercent3 = 100;
                }
          else if(moistpercent3 <=0)
                {
                  moistpercent3 = 0;
                }
          else if(moistpercent3 >0 && moistpercent3 < 100)
                {
                  moistpercent3 = moistpercent3;
                  }

          if(moistpercent4 >= 100){
                moistpercent4 = 100;
                }
          else if(moistpercent4 <=0)
                {
                  moistpercent4 = 0;
                }
          else if(moistpercent4 >0 && moistpercent4 < 100)
                {
                  moistpercent4 = moistpercent4;
                  }


    moist1 = moistpercent1;
    moist2 = moistpercent2;
    moist3 = moistpercent3;
    moist4 = moistpercent4;

//Convert the value to a char array
    char tempString[8];
    dtostrf(temp, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    client.publish("esp32/BME280/temperature", tempString);

    // Convert the value to a char array
    char humString[8];
    dtostrf(hum, 1, 2, humString);
    Serial.print("Humidity: ");
    Serial.println(humString);
    client.publish("esp32/BME280/humidity", humString);


        // Convert the value to a char array
    char PressString[8];
    dtostrf(pres, 1, 2, PressString);
    Serial.print("Humidity: ");
    Serial.println(PressString);
    client.publish("esp32/BME280/pressure", humString);

        // Convert the value to a char array
    char moistString1[8];
    dtostrf(moist1, 1, 2, moistString1);
    Serial.print("Moisture 1: ");
    Serial.println(moistString1);
    client.publish("esp32/BME280/moist1", humString);

        // Convert the value to a char array
    char moistString2[8];
    dtostrf(moist2, 1, 2, moistString2);
    Serial.print("Moisture 2: ");
    Serial.println(moistString2);
    client.publish("esp32/BME280/moist2", moistString2);

            // Convert the value to a char array
    char moistString3[8];
    dtostrf(moist3, 1, 2, moistString3);
    Serial.print("Moisture 3: ");
    Serial.println(moistString2);
    client.publish("esp32/BME280/moist2", moistString2);

             // Convert the value to a char array
    char moistString4[8];
    dtostrf(moist4, 1, 2, moistString4);
    Serial.print("Moisture 4: ");
    Serial.println(moistString4);
    client.publish("esp32/BME280/moist4", moistString4);

  }
}






  if (totalmoisture >= 65 && totalmoisture <=75 ){
    digitalWrite(pump_on,HIGH);
    digitalWrite(pump_off,LOW);
    //Serial.println("Pump working");
    pump_status = 0;

  }

    if (totalmoisture >= 80){
    digitalWrite(pump_on,LOW);
    digitalWrite(pump_off,LOW);
    //Serial.println("PUMP Off");
    pump_status = 0;
  }
   
}