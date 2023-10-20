#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>


#define I2C_ADDR 0x76
#define moist_pin1 35
#define moist_pin2 34
#define moist_pin3 39
#define moist_pin4 36
#define fan_pin_on 4
#define fan_pin_off 5
#define pump_on 18
#define pump_off 19
#define heatpin 41
#define coldpin 42
#define ledPin 43

Adafruit_BME280 bme;
const int dry = 2600;   //you need to replace this value with Value_1
const int wet = 600;  //you need to replace this value with Value_2

int pump_status, heat_status;
float moist1, moist2, moist3, moist4;
float tempF, humi, pressure, MoistVal1, MoistVal2, MoistVal3, MoistVal4;



//*** UPDATE THESE SETTINGS
// Wi-Fi settings - replace xxx with your Wi-Fi SSID and password
const char* ssid     = "1FF0 Hyperoptic 1Gb Fibre 2.4Ghz";
const char* password = "SNx5AukQHkDu";

//*** UPDATE THESE SETTINGS
//  MQTT specifics 
const char* mqttServer = "192.168.1.213";
const int mqttPort = 1883;
const char* mqttUser = "mqtt-user";
const char* mqttPassword = "1234";

WiFiClient espClient;
PubSubClient client(espClient);


void setup() {

  // Configure serial port for debug when ESP32 board is connected
  // to computer using USB port
  Serial.begin(115200);
  Serial.print("Starting setup  ");
  // Recommended delay
  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("%d...", t);
    Serial.flush();
    delay(500);
  }

  // Initiate Wi-Fi connection setup
  WiFi.begin(ssid, password);

  // Show status on serial monitor
  Serial.print("\r\nConnecting to ");
  Serial.print(ssid); Serial.print(" ...");

  // Wait for Wi-Fi connection and show progress on serial monitor - blocking
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print(" Connected! IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqttServer, mqttPort);

  // Configure and connect to BME280 Sensor via I2C - blocking
  Serial.println("Checking BME280");
  unsigned status;

  status = bme.begin(I2C_ADDR);

  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address\n");
    while (1) delay(10); //***program will hang here if no valid sensor attached***
  } else {
    Serial.println("BME280 Detected");
  }
}

// This is the main section and will loop continuously
void loop() {

  String postData;

  // This is the recommended minimum amount of time to wait before
  // reading the sensor
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

  //Following code creates the serialized JSON string to send to JEDI One
  //using ArduinoJson library v6.x
  const int capacity = JSON_OBJECT_SIZE(9);
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

  char buffer[256];
  size_t n = serializeJson(doc, buffer);

  //If MQTT client not connected, attempt connection - blocking
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
  if (client.publish("datacache/test", buffer, n) == true) {
    Serial.print("Success sending message: ");
    Serial.println(buffer);
    Serial.print("To topic: datacache/");
    //Serial.println(jediID);
  } else {
    Serial.println("Error publishing message");
  }
  client.loop();
  Serial.println("-------------");
  // Wait for 5 seconds before repeating the loop
  delay(10000);
}