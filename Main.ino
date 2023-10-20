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

int pump_status, heat_status, fan_status;
float moist1, moist2, moist3, moist4;
float tempF, humi, pressure, MoistVal1, MoistVal2, MoistVal3, MoistVal4;


String* variablesArray = nullptr;
float* floatVariablesArray = nullptr;  // Global float array
int variablesCount = 0;


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

// This is the main section and will loop continuously
void loop() {
runthings();
}





