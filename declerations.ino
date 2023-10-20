#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

// Replace the next variables with your SSID/Password combination
#define WIFI_SSID "1FF0 Hyperoptic 1Gb Fibre 2.4Ghz"
#define WIFI_PASSWORD "SNx5AukQHkDu"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 1, 164)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 5000;        // Interval at which to publish sensor readings


//define pins
#define moist_pin1 13
#define moist_pin2 12
#define moist_pin3 14
#define moist_pin4 27
#define fan_pin_on 4
#define fan_pin_off 5
#define pump_on 18
#define pump_off 19
#define heatpin 41
#define coldpin 42
#define ledPin 43


//define variables
const int dry = 2600;   //you need to replace this value with Value_1
const int wet = 600;  //you need to replace this value with Value_2

int MoistVal1, MoistVal2, MoistVal3, MoistVal4;
int moistpercent1, moistpercent2, moistpercent3, moistpercent4;


// BME280 I2C
Adafruit_BME280 bme;
// Variables to hold sensor readings
float temp;
float hum;
float pres;
float moist1, moist2, moist3, moist4;


void setup() {
   Serial.begin(115200);
  Serial.println();
  
  Serial.println("Initialize");
  Serial.println(F("BME280 test"));

  //initialize pins
  pinMode(moist_pin1, INPUT);
  pinMode(moist_pin2, INPUT);
  pinMode(moist_pin3, INPUT);
  pinMode(moist_pin4, INPUT);
  pinMode(ledPin, OUTPUT);

  bool status;

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));


  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  setup_wifi();
  //client.setServer(MQTT_HOST, 1883);
  //client.setCallback(callback);

}


void setup_wifi() {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT
  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/BME280/switch1") {
    Serial.print("Changing output to ");
    if(messageTemp == "true"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "false"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/BME280/switch1");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
  
void loop() {
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
    Serial.println("Read Value");
    Serial.println(MoistVal1);
   MoistVal2 = analogRead(moist_pin2);  //put Sensor insert into soil
   MoistVal3 = analogRead(moist_pin3);  //put Sensor insert into soil
   MoistVal4 = analogRead(moist_pin4);  //put Sensor insert into soil
    moistpercent1 = map(MoistVal1, dry, wet, 100, 0);
    Serial.println("Map Value");
    Serial.println(moistpercent1);
   moistpercent2 = map(MoistVal2, dry, wet, 100, 0);
    moistpercent3 = map(MoistVal3, dry, wet, 100, 0);
    moistpercent4 = map(MoistVal4, dry, wet, 100, 0);
         
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
    //  moist2 = moistpercent2;
     // moist3 = moistpercent3;
     // moist4 = moistpercent4;

Convert the value to a char array
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
}
