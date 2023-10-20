void heatloop(){
  if (tempF <= 20){
    digitalWrite(heatpin,HIGH);
    digitalWrite(coldpin,LOW);
    //Serial.println("Heater ON");
    heat_status = 1;

  }

  else if (tempF >= 45){
    digitalWrite(heatpin,LOW);
    digitalWrite(coldpin,HIGH);
    //Serial.println("Heater Off");
    heat_status = 0;
  }

  else if (tempF <= 30 && tempF >= 40){
    digitalWrite(heatpin,LOW);
    digitalWrite(coldpin,LOW);
  }
}


void moistureloop(){
  int totalmoisture = (moist1 + moist2+ moist3 + moist4)/4;
  Serial.println(totalmoisture);
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
