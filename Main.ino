 //Include Libraries.
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <sunset.h>
#include <time.h>

//Define Pins
#define I2C_ADDR 0x76
#define moist_pin1 35
#define moist_pin2 34
#define moist_pin3 39
#define moist_pin4 36
#define fanpin_on 16
#define fanpin_off 17
#define pump_on 18
#define pump_off 19
#define heatpin_on 32
#define heatpin_off 33
#define lightpin 23
#define TIMEZONE +1
#define LATITUDE 51.5072
#define LONGITUDE 0.1276


//Define Floats and Integers 
Adafruit_BME280 bme;
const int dry = 2600;   //you need to replace this value with Value_1
const int wet = 600;  //you need to replace this value with Value_2


const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;


unsigned long lastToggleTime = 0;

int pump_status, heat_status, fan_status, bulb_status, manual_status;
int pump_bool, heat_bool, fan_bool, manual_bool, select_bool, temp_bool, bulb_bool = false;
float moist1, moist2, moist3, moist4;
float tempF, humi, pressure, MoistVal1, MoistVal2, MoistVal3, MoistVal4;

String* variablesArray = nullptr;
float* floatVariablesArray = nullptr;  // Global float array
int variablesCount = 0;


// Wi-Fi settings - replaced xxx with Wi-Fi SSID and password
const char* ssid     = "Redmi Note 10 Pro";
const char* password = "12344321";
//  MQTT specifics 
const char* mqttServer = "192.168.12.219";
const int mqttPort = 1883;
const char* mqttUser = "mqtt-user";
const char* mqttPassword = "1234";

//Object Creation
WiFiClient espClient;
PubSubClient client(espClient);
SunSet sun;

//MISC Variables
long lastMsg = 0;
char msg[50];
int value = 0;


//ESP32 Output Message Creation
void callback(char* topic, byte* message, unsigned int length) {

  String messageTemp;
    
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);
  int Temp = messageTemp.toInt();
  Serial.println(Temp);

  String temptopic(topic);
  Serial.println(temptopic); 

  if (temptopic == "homeassist/manual" &&  Temp == 1)
  {
    manual_bool = 1;
    Serial.println("Manual bool turned on");
  }
  
  else if (temptopic == "homeassist/manual" &&  Temp == 0) 
  {
    manual_bool = 0;
    Serial.println("manual bool turned off");
  }
  

  if ((temptopic == "homeassist/heater") && (Temp == 1))
  {
    heat_bool = 1;
    Serial.println("Heater bool on"); 
  }
  else if (temptopic == "homeassist/heater" && Temp == 0) {
    heat_bool = 0;
    Serial.println("Heater bool off");
  }


  if (temptopic == "homeassist/pump" &&  Temp == 1)
  {
    pump_bool = 1; 
    Serial.println("pump bool on");
  }
  else if (temptopic == "homeassist/pump" &&  Temp == 0) {
    pump_bool = 0;
    Serial.println("pump bool off");
  }


  if ((temptopic == "homeassist/fan") && (Temp == 1))
  {
    fan_bool = 1;
    Serial.println("fan bool on"); 
  }
  else if (temptopic == "homeassist/fan" && Temp == 0) {
    fan_bool = 0;
    Serial.println("fan bool off");
  }
}


void setup() {
  // Configure serial port for debug when ESP32 board is connected
  Serial.begin(115200);
  Serial.print("Starting setup  ");
  
  // Recommended delay
  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("%d...", t);
    Serial.flush();
    delay(500);
  }

  //Sync ESP32 rtc time with real world time and get sunset and sunrise time based on location
  struct tm timeinfo;
  sun.setPosition(LATITUDE, LONGITUDE, TIMEZONE); 
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  sun.setCurrentDate(timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday);

  //pin type decleration
  pinMode(moist_pin1,INPUT);
  pinMode(moist_pin2,INPUT);
  pinMode(moist_pin3,INPUT);
  pinMode(moist_pin4,INPUT);
  pinMode(fanpin_on,OUTPUT);
  pinMode(fanpin_off,OUTPUT);
  pinMode(pump_on,OUTPUT);
  pinMode(pump_off,OUTPUT);
  pinMode(heatpin_on,OUTPUT);
  pinMode(heatpin_off,OUTPUT);
  pinMode(lightpin,OUTPUT);

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
  //Connect to MQTT 
  client.setServer(mqttServer, mqttPort);
  client.connect("core-mosquitto", "mqtt-user", "1234");
  client.subscribe("homeassist/heater");
  client.subscribe("homeassist/pump");
  client.subscribe("homeassist/fan");
  client.subscribe("homeassist/manual");
  client.setCallback(callback);
  

  // Configure and connect to BME280 Sensor via I2C - blocking
  Serial.println("Checking BME280");
  unsigned status;

  status = bme.begin(I2C_ADDR);

  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address\n");
    while (1) delay(10); //program will hang here if no valid sensor attached
  } else {
    Serial.println("BME280 Detected");
  }

  //Collect data from a web server file.
  HTTPClient http;
  http.begin("https://raw.githubusercontent.com/MuhammadMohtashim/Vertical-Farming/main/data.csv");
  int httpCode = http.GET();
  if (httpCode > 0) {
    String payload = http.getString();
    int rows = 0;
    for (int i = 0; i < payload.length(); i++) {
      if (payload[i] == '\n') {
        rows++;
      }
    }
    variablesArray = new String[rows];
    variablesCount = rows;
    int stringStart = 0;
    int stringEnd = 0;
    for (int r = 0; r < rows; r++) {
      stringEnd = payload.indexOf('\n', stringStart);
      String row = payload.substring(stringStart, stringEnd);
      int colIndex = row.indexOf(',');
      variablesArray[r] = row.substring(colIndex + 1);
      stringStart = stringEnd + 1;
    }
  }
  else { 
    Serial.println("Error on HTTP request");
  }
  http.end();
  floatVariablesArray = new float[variablesCount];  // Initialize float array
  for (int i = 0; i < variablesCount; i++) {
    floatVariablesArray[i] = variablesArray[i].toFloat();  // Convert String to float
  }
}


void loop() {

runthings();

}


