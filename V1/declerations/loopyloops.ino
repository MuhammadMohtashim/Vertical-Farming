void moisture_loop (){
  int soilmoisturepercent_total= (soilmoisturepercent1 + soilmoisturepercent2 + soilmoisturepercent3 + soilmoisturepercent4)/4;
  if (soilmoisturepercent_total <= 75){
    digitalWrite(pump_pin_on, HIGH);
    digitalWrite(pump_pin_off, LOW);
    Serial.println("pump on");
     
  }

  else if (soilmoisturepercent_total >= 95){
    digitalWrite(pump_pin_on, LOW);
    digitalWrite(pump_pin_on, LOW);
    Serial.println("pump off") ;

  }
  Serial.println("moisture loop");
}


void heat_loop(){
  if (temp <=20){
    digitalWrite(heat_pin, HIGH);
    digitalWrite(cold_pin, LOW);
    Serial.println("heater on");
  }

  else if (temp >= 40){
    digitalWrite(heat_pin, LOW);
    digitalWrite(cold_pin, HIGH);
    Serial.println("fan on");

  }
  Serial.println("heat loop");

}

