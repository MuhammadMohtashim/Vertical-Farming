void runthings(){

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastToggleTime;
  
  if (bulb_bool && elapsedTime >= 28800000) { // 8 hours in milliseconds
    digitalWrite(lightpin, LOW);
    bulb_bool = false;
    bulb_status = 0;
    lastToggleTime = currentTime;
    Serial.println("Bulb is OFF");
  } 
  else if (!bulb_bool && elapsedTime >= 57600000) { // 16 hours in milliseconds
    digitalWrite(lightpin, HIGH);
    bulb_bool = true;
    bulb_status = 1;
    lastToggleTime = currentTime;
    Serial.println("Bulb is ON");
  }
  // This is the recommended minimum amount of time to wait before reading sensor
  delay(1000);

  String postData;
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
  moist1 = constrain(moistpercent1, 0, 100);
  moist2 = constrain(moistpercent2, 0, 100);
  moist3 = constrain(moistpercent3, 0, 100);
  moist4 = constrain(moistpercent4, 0, 100);

  if (manual_bool == 1){
    manual_status = 1;
    if (heat_bool == 1)
    {
      digitalWrite(heatpin_on,HIGH);
      digitalWrite(heatpin_off,LOW);
      heat_status = 1;
      Serial.println("Heater on");
    }
    else if (heat_bool == 0){
      digitalWrite(heatpin_on,LOW);
      digitalWrite(heatpin_off,LOW);
      heat_status = 0;
      Serial.println("Heater off");
    }

    if (pump_bool == 1){
      digitalWrite(pump_on,HIGH);
      digitalWrite(pump_off,LOW);
      pump_status = 1;
      Serial.println("pump on");
    }
    else if (pump_bool == 0){
      digitalWrite(pump_on,LOW);
      digitalWrite(pump_off,LOW);
      Serial.println("pump off");
      pump_status = 0;
    }

    if (fan_bool == 1){
      digitalWrite(fanpin_on,HIGH);
      digitalWrite(fanpin_off,LOW);
      Serial.println("fan on");
      fan_status = 1;
    }
    else if(fan_bool == 0){
      digitalWrite(fanpin_on,LOW);
      digitalWrite(fanpin_off,LOW);
      Serial.println("fan off");
      fan_status = 0;
    }
  }

  if (manual_bool == 0) {
    //Serial.println("Manual switch de-activated");
    heatloop();
    moistureloop();
    manual_status = 0;
  }


  //Creates serialized JSON string to send to Home Assistant
  const int capacity = JSON_OBJECT_SIZE(12);
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
  doc["ManualStat"] = manual_status;
  doc["BulbStat"] = bulb_status;

  char buffer[256];
  size_t n = serializeJson(doc, buffer);

  //If MQTT client not connected, attempt connection
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
     if (client.connect("core-mosquitto", mqttUser, mqttPassword )) {
      Serial.println("connected");
    } 
     else {
      Serial.print("failed with state ");
      Serial.println(client.state());
      delay(5000);
    }
  }

  Serial.println("\r\n-------------");
  //publish MQTT message with BME280 payload
  if (client.publish("esp32/output", buffer, n) == true) {
   // Serial.print("Success sending message: ");
    Serial.println(buffer);
   // Serial.print("To topic: esp32/");
  } else {
    Serial.println("Error publishing message");
  }
  client.loop();
  //Serial.println("-------------");
  // Wait for 5 seconds before repeating the loop
  delay(1000);
}



void heatloop(){ //Whether to turn on heater or fan

  if (tempF <= floatVariablesArray[0] && (heat_bool == 1 || heat_bool == 0)){
    //Serial.println(floatVariablesArray[1]);
    digitalWrite(heatpin_on,HIGH);
    digitalWrite(heatpin_off,LOW);
    Serial.println("Heater ON from Values");
    heat_status = 1;
    fan_status = 0;
  }

  else if (tempF >= floatVariablesArray[1] && (fan_bool == 1 || fan_bool == 0)){
    //Serial.println(floatVariablesArray[0]);
    digitalWrite(fanpin_on,HIGH);
    digitalWrite(fanpin_off,LOW);
    digitalWrite(heatpin_on,LOW);
    digitalWrite(heatpin_off,LOW);
    Serial.println("Heater Off from and Fan ON Values");
    heat_status = 0;
    fan_status = 1;
  }

  else if (tempF > floatVariablesArray[0] && tempF < floatVariablesArray[1] && (heat_bool == 1 || heat_bool == 0 || fan_bool == 1 || fan_bool == 0)){
    digitalWrite(heatpin_on,LOW);
    digitalWrite(heatpin_off,LOW);
    digitalWrite(fanpin_on,LOW);
    digitalWrite(fanpin_off,LOW);
    heat_status = 0;
    fan_status = 0;
  }
}


void moistureloop(){//Whether to turn on pump or not
  int totalmoisture = (moist1 + moist2+ moist3 + moist4)/4;
  Serial.println("Total Moisture");
  Serial.println(totalmoisture);
  //Serial.println(totalmoisture);

  if (totalmoisture <= floatVariablesArray[4] && (pump_bool == 1 || pump_bool == 0)){
    digitalWrite(pump_on,HIGH);
    digitalWrite(pump_off,LOW);
    Serial.println("PUMP ON from Values");
    pump_status = 1;
  }

  if (totalmoisture > floatVariablesArray[4] && totalmoisture < floatVariablesArray[5] && (pump_bool == 1 || pump_bool == 0) ){
    digitalWrite(pump_on,HIGH);
    digitalWrite(pump_off,LOW);
    Serial.println("Pump Filling working from Values");
    pump_status = 0;
  }

    if (totalmoisture >= floatVariablesArray[5] && (pump_bool == 1 || pump_bool == 0)){
    digitalWrite(pump_on,LOW);
    digitalWrite(pump_off,LOW);
    Serial.println("PUMP Off from Values");
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
