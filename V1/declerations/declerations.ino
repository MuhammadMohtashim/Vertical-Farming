#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PubSubClient.h>
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define WIFI_SSID "1FF0 Hyperoptic 1Gb Fibre 2.4Ghz"
#define WIFI_PASSWORD "SNx5AukQHkDu"

#define moisture_pin1 13
#define moisture_pin2 12
#define moisture_pin3 14
#define moisture_pin4 27
#define fan_pin_on 4
#define fan_pin_off 5
#define pump_pin_on 18
#define pump_pin_off 19
#define heat_pin 41
#define cold_pin 42
#define ledPin 43


const int dry = 2600;   //you need to replace this value with Value_1
const int wet = 600;  //you need to replace this value with Value_2
int soilMoistureValue1 = 0;
int soilMoistureValue2 = 0;
int soilMoistureValue3 = 0;
int soilMoistureValue4 = 0;


int soilmoisturepercent1=0;
int soilmoisturepercent2=0;
int soilmoisturepercent3=0;
int soilmoisturepercent4=0;



// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 1, 233)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883
#define MQTT_User "mqtt-user"
#define MQTT_Pass "1234"


// Temperature MQTT Topics
#define MQTT_PUB_TEMP "esp32/bme280/temperature"
#define MQTT_PUB_HUM "esp32/bme280/humidity"
#define MQTT_PUB_PRES "esp32/bme280/pressure"
#define MQTT_PUB_MOIST1 "esp32/bme280/moisture1"
#define MQTT_PUB_MOIST2 "esp32/bme280/moisture2"
#define MQTT_PUB_MOIST3 "esp32/bme280/moisture3"
#define MQTT_PUB_MOIST4 "esp32/bme280/moisture4"

// BME280 I2C
Adafruit_BME280 bme;
// Variables to hold sensor readings
float temp;
float hum;
float pres;
float moist1, moist2, moist3, moist4;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 5000;        // Interval at which to publish sensor readings




void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent ) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}


void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
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
  if (String(topic) == "esp32/BME280/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "true"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
      Serial.println("LED HIGH");
    }
    else if(messageTemp == "false"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
      Serial.println("LED LOW");
    }

    else if(messageTemp != "false" && messageTemp != "true"){
      Serial.println("Not working");
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  Serial.println("Initialize");
  Serial.println(F("BME280 test"));

  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();
  client.connect("arduinoClient", MQTT_User, MQTT_Pass);
  client.setCallback(callback);
}

void loop() {

postthings();

}