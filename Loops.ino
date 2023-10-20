void runthings(){

  if (!client.connected()) {
    reconnect();
  }
  else  {
    Serial.println("Topic Subscribed");
  }

  client.loop();

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastToggleTime;
  
  if (bulbState && elapsedTime >= 28800000) { // 8 hours in milliseconds
    digitalWrite(lightpin, LOW);
    bulbState = false;
    lastToggleTime = currentTime;
    Serial.println("Bulb is OFF");
  } 
  else if (!bulbState && elapsedTime >= 57600000) { // 16 hours in milliseconds
    digitalWrite(lightpin, HIGH);
    bulbState = true;
    lastToggleTime = currentTime;
    Serial.println("Bulb is ON");
  }

  String postData;

  // This is the recommended minimum amount of time to wait before reading sensor
  delay(1000);



  // Read the sensor, convert readings and set variables
  tempF = bme.readTemperature();
  humi = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  MoistVal1 = analogRead(moist_pin1);  //put Sensor insert into soil
  MoistVal2 = analogRead(moist_pin2);  //put Sensor insert into soil
  MoistVal3 = analogRead(moist_pin3);  //put Sensor insert into soil
  MoistVal4 = analogRead(moist_pin4);  //put Sensor insert into soil

  int moistpercent1, moistpercent2, moistpercent3, moistpercent4;

  moistpercent1 = map(MoistVal1, dry, wet, 0, 100);
  moistpercent2 = map(MoistVal2, dry, wet, 0, 100);
  moistpercent3 = map(MoistVal3, dry, wet, 0, 100);
  moistpercent4 = map(MoistVal4, dry, wet, 0, 100);

  if(moistpercent1 >= 100)
          {
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

  if(moistpercent2 >= 100)
          {
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

  if(moistpercent3 >= 100)
          {
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

  if(moistpercent4 >= 100)
          {
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

  heatloop();
  moistureloop();

  //Creates serialized JSON string to send to Home Assistant
  const int capacity = JSON_OBJECT_SIZE(10);
  StaticJsonDocument<capacity> doc;

  doc["tempF"] = tempF;
  doc["humi"] = humi;
  doc["pressure"] = pressure;
  doc["moist1"] = moist1;
  doc["moist2"] = moist2; 
  doc["moist3"] = moist3;
  doc["moist4"] = moist4;
  doc["PumpStat"] = pump_status;
  doc["HeatStat"] = heat_status;
  doc["FanStat"] = fan_status;

  char buffer[256];
  size_t n = serializeJson(doc, buffer);

  //If MQTT client not connected, attempt connection
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
     if (client.connect("core-mosquitto", mqttUser, mqttPassword )) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.println(client.state());
      delay(5000);
    }
  }
  Serial.println("\r\n-------------");
  //publish MQTT message with BME280 payload
  if (client.publish("esp32/output", buffer, n) == true) {
    Serial.print("Success sending message: ");
    Serial.println(buffer);
    Serial.print("To topic: esp32/");
    //Serial.println(jediID);
  } else {
    Serial.println("Error publishing message");
  }
  client.loop();
  Serial.println("-------------");
  // Wait for 5 seconds before repeating the loop
  delay(10000);
}


void heatloop(){ //Whether to turn on heater or fan
  if (tempF <= floatVariablesArray[0]){
    digitalWrite(heatpin,HIGH);
    digitalWrite(coldpin,LOW);
    //Serial.println("Heater ON");
    heat_status = 1;
    fan_status = 0;

  }

  else if (tempF >= floatVariablesArray[1]){
    digitalWrite(heatpin,LOW);
    digitalWrite(coldpin,HIGH);
    //Serial.println("Heater Off");
    heat_status = 0;
    fan_status = 1;
  }

  else if (tempF <= floatVariablesArray[0] && tempF >= floatVariablesArray[1]){
    digitalWrite(heatpin,LOW);
    digitalWrite(coldpin,LOW);
    heat_status = 0;
    fan_status = 0;
  }
}


void moistureloop(){//Whether to turn on pump or not
  int totalmoisture = (moist1 + moist2+ moist3 + moist4)/4;
  //Serial.println(totalmoisture);
  if (totalmoisture <= 60){
    digitalWrite(pump_on,HIGH);
    digitalWrite(pump_off,LOW);
    //Serial.println("PUMP ON");
    pump_status = 1;
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

int countLines(String payload) { //String to Integer
  int lines = 0;
  for (int i = 0; i < payload.length(); i++) {
    if (payload[i] == '\n') {
      lines++;
    }
  }
  return lines;
}

void parseCSV(String payload) { //Parsing of data from CSV File
  int lineIndex = 0;
  int dataIndex = 0;
  String variable = "";
  for (int i = 0; i < payload.length(); i++) {
    char c = payload[i];
    if (c == '\n') {
      if (variable != "") {
        variablesArray[lineIndex] = variable;
        Serial.print("Global Variable at line ");
        Serial.print(lineIndex);
        Serial.print(": ");
        Serial.println(variable);
      }
      lineIndex++;
      dataIndex = 0;
      variable = "";
    } else if (c == ',') {
      dataIndex++;
      if (dataIndex == 1) {
        variable = "";
      }
    } else if (dataIndex == 1) {
      variable += c;
    }
  }
}
