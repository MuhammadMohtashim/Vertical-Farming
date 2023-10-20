#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>

//define pins
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

//define variables
const int dry = 2600;   //you need to replace this value with Value_1
const int wet = 600;  //you need to replace this value with Value_2

int MoistVal1, MoistVal2, MoistVal3, MoistVal4;
int moistpercent1, moistpercent2, moistpercent3, moistpercent4;

// Replace the next variables with your SSID/Password combination
const char* ssid = "1FF0 Hyperoptic 1Gb Fibre 2.4Ghz";
const char* password = "SNx5AukQHkDu";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.1.213";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
char* topic; 
byte* message; 
unsigned int length;


// BME280 I2C
Adafruit_BME280 bme;
// Variables to hold sensor readings
float temp;
float hum;
float pres;
float moist1, moist2, moist3, moist4;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 5000;        // Interval at which to publish sensor readings

void setup() {
  Serial.begin(115200);
 bool status;


  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.connect("core-mosquitto", "mqtt-user", "1234");
  client.setCallback(callback);

    //initialize pins
  pinMode(moist_pin1, INPUT);
  pinMode(moist_pin2, INPUT);
  pinMode(moist_pin3, INPUT);
  pinMode(moist_pin4, INPUT);
  pinMode(ledPin, OUTPUT);


}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback() {
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
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
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
      client.subscribe("esp32/output");
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
postthings();
}