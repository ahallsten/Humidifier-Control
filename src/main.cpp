/*LIBRARIES*/
/*===========================================*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_INA3221.h"
#include <ESP8266WiFi.h>
// #include <PubSubClient.h>

/*DEFINITIONS & VARIABLES*/
/*===========================================*/
// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// INA3221 sensor
Adafruit_INA3221 ina3221;

// WiFi and MQTT Configuration
const char *ssid = "Hallsten";
const char *password = "Spectrum600!";
// const char *mqtt_server = "Your_MQTT_BROKER_IP";
// const char *mqtt_user = "Your_MQTT_USERNAME";
// const char *mqtt_password = "Your_MQTT_PASSWORD";

// // MQTT Topics
// const char *tempTopic = "home/humidity_controller/temperature";
// const char *rhTopic = "home/humidity_controller/humidity";
// const char *pressureTopic = "home/humidity_controller/pressure";
// const char *relayTopic = "home/humidity_controller/relay_state";

// MQTT Client
WiFiClient espClient;
// PubSubClient client(espClient);

// Pins
#define BLOWER_FAN_PIN D1
#define HUMIDIFIER_RELAY_PIN D2

// Calibration
#define TEMP_MIN_MA 4.0
#define TEMP_MAX_MA 20.0
#define TEMP_MIN_F 32.0
#define TEMP_MAX_F 212.0

#define RH_MIN_MA 4.0
#define RH_MAX_MA 20.0
#define RH_MIN 0.0
#define RH_MAX 100.0

#define PRESSURE_MIN_MA 4.0
#define PRESSURE_MAX_MA 20.0
#define PRESSURE_MIN_PSI 0.0
#define PRESSURE_MAX_PSI 100.0

// Deadband and Setpoint
float humiditySetpoint = 50.0;
float deadband = 3.0;

// Global Variables
float temperature = 0.0;
float humidity = 0.0;
float pressure = 0.0;
bool relayState = false;
unsigned long lastRelayToggleTime = 0;
const unsigned long relayToggleDelay = 30000;

// Timing variables
unsigned long lastSensorReadTime = 0;
unsigned long lastMqttPublishTime = 0;
unsigned long lastDisplayUpdateTime = 0;
const unsigned long sensorReadInterval = 100;
const unsigned long mqttPublishInterval = 5000;
const unsigned long displayUpdateInterval = 200;

// Moving average buffers
#define BUFFER_SIZE 20
float tempBuffer[BUFFER_SIZE] = {0};
float rhBuffer[BUFFER_SIZE] = {0};
float pressureBuffer[BUFFER_SIZE] = {0};
int bufferIndex = 0;

/*FUNCTION DECLARATIONS*/
/*===========================================*/
void setupWiFi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected!");
}

// void reconnectMQTT()
// {
//   while (!client.connected())
//   {
//     Serial.print("Attempting MQTT connection...");
//     if (client.connect("ESP8266Client", mqtt_user, mqtt_password))
//     {
//       Serial.println("connected");
//     }
//     else
//     {
//       Serial.print("failed, rc=");
//       Serial.print(client.state());
//       Serial.println(" try again in 5 seconds");
//       delay(5000);
//     }
//   }
// }

/*SETUP*/
/*===========================================*/
void setup()
{
  pinMode(BLOWER_FAN_PIN, INPUT);
  pinMode(HUMIDIFIER_RELAY_PIN, OUTPUT);
  digitalWrite(HUMIDIFIER_RELAY_PIN, LOW);

  Serial.begin(115200);
  setupWiFi();
  // client.setServer(mqtt_server, 1883);

  if (!display.begin(SSD1306_PAGEADDR, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    while (true)
      Serial.println(F("shit"));
    ;
  }
  display.clearDisplay();
  display.display();

  if (!ina3221.begin())
  {
    Serial.println("Failed to initialize INA3221!");
    while (true)
      ;
  }
}

float convertToRange(float input, float inputMin, float inputMax, float outputMin, float outputMax)
{
  return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
}

float calculateAverage(float *buffer, int size)
{
  float sum = 0;
  for (int i = 0; i < size; i++)
  {
    sum += buffer[i];
  }
  return sum / size;
}

void readSensors()
{
  float tempCurrent = ina3221.getCurrentAmps(1);     // Channel 1
  float rhCurrent = ina3221.getCurrentAmps(2);       // Channel 2
  float pressureCurrent = ina3221.getCurrentAmps(3); // Channel 3

  tempBuffer[bufferIndex] = convertToRange(tempCurrent, TEMP_MIN_MA, TEMP_MAX_MA, TEMP_MIN_F, TEMP_MAX_F);
  rhBuffer[bufferIndex] = convertToRange(rhCurrent, RH_MIN_MA, RH_MAX_MA, RH_MIN, RH_MAX);
  pressureBuffer[bufferIndex] = convertToRange(pressureCurrent, PRESSURE_MIN_MA, PRESSURE_MAX_MA, PRESSURE_MIN_PSI, PRESSURE_MAX_PSI);

  temperature = calculateAverage(tempBuffer, BUFFER_SIZE);
  humidity = calculateAverage(rhBuffer, BUFFER_SIZE);
  pressure = calculateAverage(pressureBuffer, BUFFER_SIZE);

  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
}

// void publishToMQTT()
// {
//   if (!client.connected())
//   {
//     reconnectMQTT();
//   }
//   client.loop();

//   client.publish(tempTopic, String(temperature).c_str(), true);
//   client.publish(rhTopic, String(humidity).c_str(), true);
//   client.publish(pressureTopic, String(pressure).c_str(), true);
//   client.publish(relayTopic, relayState ? "ON" : "OFF", true);
// }

/*LOOP*/
/*===========================================*/
void loop()
{
  unsigned long currentTime = millis();

  if (currentTime - lastSensorReadTime >= sensorReadInterval)
  {
    lastSensorReadTime = currentTime;
    readSensors();
  }

  // if (currentTime - lastMqttPublishTime >= mqttPublishInterval)
  // {
  //   lastMqttPublishTime = currentTime;
  //   publishToMQTT();
  // }

  if (currentTime - lastDisplayUpdateTime >= displayUpdateInterval)
  {
    lastDisplayUpdateTime = currentTime;
    // Update display code here...
  }
}