

void mqtt_post(){
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

    soilMoistureValue1 = analogRead(moisture_pin1);  //put Sensor insert into soil
    soilMoistureValue2 = analogRead(moisture_pin2);  //put Sensor insert into soil
    soilMoistureValue3 = analogRead(moisture_pin3);  //put Sensor insert into soil
    soilMoistureValue4 = analogRead(moisture_pin4);  //put Sensor insert into soil
    soilMoistureValue5 = analogRead(moisture_pin5);  //put Sensor insert into soil
    soilmoisturepercent1 = map(soilMoistureValue1, AirValue, WaterValue, 0, 100);
    soilmoisturepercent2 = map(soilMoistureValue2, AirValue, WaterValue, 0, 100);
    soilmoisturepercent3 = map(soilMoistureValue3, AirValue, WaterValue, 0, 100);
    soilmoisturepercent4 = map(soilMoistureValue4, AirValue, WaterValue, 0, 100);
    soilmoisturepercent5 = map(soilMoistureValue5, AirValue, WaterValue, 0, 100);
          
          if(soilmoisturepercent1 >= 100){
                soilmoisturepercent1 = 100;
                }
                else if(soilmoisturepercent1 <=0)
                {
                  soilmoisturepercent1 = 0;
                }
                else if(soilmoisturepercent1 >0 && soilmoisturepercent1 < 100)
                {
                  soilmoisturepercent1 = soilmoisturepercent1;
                  }

                  if(soilmoisturepercent2 >= 100){
                soilmoisturepercent2 = 100;
                }
                else if(soilmoisturepercent2 <=0)
                {
                  soilmoisturepercent2 = 0;
                }
                else if(soilmoisturepercent2 >0 && soilmoisturepercent2 < 100)
                {
                  soilmoisturepercent2 = soilmoisturepercent2;
                  }

                   if(soilmoisturepercent3 >= 100){
                soilmoisturepercent3 = 100;
                }
                else if(soilmoisturepercent3 <=0)
                {
                  soilmoisturepercent3 = 0;
                }
                else if(soilmoisturepercent3 >0 && soilmoisturepercent3 < 100)
                {
                  soilmoisturepercent3 = soilmoisturepercent3;
                  }


                   if(soilmoisturepercent4 >= 100){
                soilmoisturepercent4 = 100;
                }
                else if(soilmoisturepercent4 <=0)
                {
                  soilmoisturepercent4 = 0;
                }
                else if(soilmoisturepercent4 >0 && soilmoisturepercent4 < 100)
                {
                  soilmoisturepercent4 = soilmoisturepercent4;
                  }

                   if(soilmoisturepercent5 >= 100){
                soilmoisturepercent5 = 100;
                }
                else if(soilmoisturepercent5 <=0)
                {
                  soilmoisturepercent5 = 0;
                }
                else if(soilmoisturepercent5 >0 && soilmoisturepercent5 < 100)
                {
                  soilmoisturepercent5 = soilmoisturepercent5;
                  }

      moist1 = soilmoisturepercent1;
      moist2 = soilmoisturepercent2;
      moist3 = soilmoisturepercent3;
      moist4 = soilmoisturepercent4;
      moist5 = soilmoisturepercent5;
    
    // Publish an MQTT message on topic esp32/BME2800/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Message: %.2f \n", temp);

    // Publish an MQTT message on topic esp32/BME2800/humidity
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
    Serial.printf("Message: %.2f \n", hum);

    // Publish an MQTT message on topic esp32/BME2800/pressure
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_PRES, 1, true, String(pres).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_PRES, packetIdPub3);
    Serial.printf("Message: %.3f \n", pres);

    // Publish an MQTT message on topic esp32/BME2800/moisture
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_MOIST1, 1, true, String(moist1).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_MOIST1, packetIdPub4);
    Serial.printf("Message: %.3f \n", moist1);

    uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_MOIST2, 1, true, String(moist2).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_MOIST2, packetIdPub5);
    Serial.printf("Message: %.3f \n", moist2);

    uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_MOIST3, 1, true, String(moist3).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_MOIST3, packetIdPub6);
    Serial.printf("Message: %.3f \n", moist3);

    uint16_t packetIdPub7 = mqttClient.publish(MQTT_PUB_MOIST4, 1, true, String(moist4).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_MOIST4, packetIdPub7);
    Serial.printf("Message: %.3f \n", moist4);

    uint16_t packetIdPub8 = mqttClient.publish(MQTT_PUB_MOIST5, 1, true, String(moist5).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_MOIST5, packetIdPub8);
    Serial.printf("Message: %.3f \n", moist5);
    
  }
}
