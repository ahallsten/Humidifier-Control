/*LIBRARIES*/
/*===========================================*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_AM2315.h>
// #include "Adafruit_INA3221.h"
#include <ESP8266WiFi.h>
// #include <PubSubClient.h>

/*DEFINITIONS & VARIABLES*/
/*===========================================*/
// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// // INA3221 sensor
// Adafruit_INA3221 ina3221;

// ADS1115 ADS
Adafruit_ADS1115 ads; /* Use this for the 16-bit version */

// AM2315 Temp and RH Sensor
Adafruit_AM2315 am2315;

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
#define BLOWER_FAN_PIN D4
#define HUMIDIFIER_RELAY_PIN D3
#define BUTTON_PIN D5
// #define ALRT_READY D6

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

#define TEMP_CAL_OFFSET -3.088
#define RH_CAL_OFFSET 0.0
#define PRESSURE_CAL_OFFSET 0.0

// Deadband and Setpoint
float humiditySetpoint = 50.0;
float deadband = 3.0;

// Global Variables
float temperature = 0.0;
float humidity = 0.0;
float pressure = 0.0;
float am2315Temperature = 0;
float am2315humidity = 0;
bool relayState = false;
unsigned long lastRelayToggleTime = 0;
const unsigned long relayToggleDelay = 30000;

// Timing variables
unsigned long lastSensorReadTime = 0;
unsigned long lastMqttPublishTime = 0;
unsigned long lastDisplayUpdateTime = 0;
unsigned long lastSerialUpdateTime = 0;
unsigned long lastAM2315ReadTime = 0;
const unsigned long sensorReadInterval = 100;
const unsigned long mqttPublishInterval = 5000;
const unsigned long displayUpdateInterval = 200;
const unsigned long SerialUpdateInterval = 20;
const unsigned long AM2315ReadInterval = 600;

// Moving average buffers
#define BUFFER_SIZE 40
float tempBuffer[BUFFER_SIZE] = {0};
float rhBuffer[BUFFER_SIZE] = {0};
float pressureBuffer[BUFFER_SIZE] = {0};
int bufferIndex = 0;
float tempCurrent = 0;
float rhCurrent = 0;
float pressureCurrent = 0;
float tempCurrentRaw = 0;
float rhCurrentRaw = 0;
float pressureCurrentRaw = 0;
float current3 = 0;
int16_t adc0, adc1, adc2, adc3;
float volts0, volts1, volts2, volts3;

// ISR Variables
volatile bool rawRead = false;               // Variable to toggle on button press
volatile unsigned long lastDebounceTime = 0; // Tracks the last debounce time
const unsigned long debounceDelay = 40;      // Debounce time in milliseconds
volatile bool new_data = false;              // Flag for new ADS Data Ready

/*SETUP FUNCTION DECLARATIONS*/
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

// Interrupt Service Routine (ISR) for button press
void IRAM_ATTR handleButtonPress()
{
  unsigned long currentTime = millis();
  // Check if debounce time has passed
  if (currentTime - lastDebounceTime > debounceDelay)
  {
    rawRead = !rawRead;             // Toggle the state of rawRead
    lastDebounceTime = currentTime; // Update the last debounce time
  }
}

// void IRAM_ATTR NewDataReadyISR()
// {
//   new_data = true;
// }

/*SETUP*/
/*===========================================*/
void setup()
{
  pinMode(BLOWER_FAN_PIN, INPUT);
  pinMode(HUMIDIFIER_RELAY_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Use internal pull-up resistor
  // pinMode(ALRT_READY, INPUT_PULLUP);
  digitalWrite(HUMIDIFIER_RELAY_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);
  // attachInterrupt(digitalPinToInterrupt(ALRT_READY), NewDataReadyISR, FALLING);

  Serial.begin(115200);
  setupWiFi();
  // client.setServer(mqtt_server, 1883);

  // INA3221 Setup
  // if (!ina3221.begin())
  // {
  //   Serial.println("Failed to initialize INA3221!");
  //   while (true)
  //     ;
  // }
  // ina3221.setShuntResistance(0, 120);
  // ina3221.setShuntResistance(1, 120);
  // ina3221.setShuntResistance(2, 120);
  // ina3221.setAveragingMode(INA3221_AVG_4_SAMPLES);
  // ina3221.setShuntVoltageConvTime(INA3221_CONVTIME_8MS);

  // ADS1115 Setup
  if (!ads.begin())
  {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }
  ads.setDataRate(RATE_ADS1115_860SPS);

  // AM2315 Setup
  Serial.println("AM2315 Test!");
  if (!am2315.begin())
  {
    Serial.println("Sensor not found, check wiring & pullups!");
    while (1)
      ;
  }
  delay(2000);

  Serial.println(F("SSD Setup"));
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    while (true)
      Serial.println(F("shit"));
    ;
  }
  display.display();
  delay(2000);
  display.clearDisplay();
  Serial.println(F("finished SSD1306 setup"));
}

/*LOOP FUNCTION DECLARATIONS*/
/*===========================================*/
float analogScaling(float input, float inputMin, float inputMax, float outputMin, float outputMax, float calOffset)
{
  return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin + calOffset;
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
  // tempCurrent = 1000 * ina3221.getCurrentAmps(0);     // Channel 1
  // rhCurrent = 1000 * ina3221.getCurrentAmps(1);       // Channel 2
  // pressureCurrent = 1000 * ina3221.getCurrentAmps(2); // Channel 3

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);

  volts0 = ads.computeVolts(adc0);
  volts1 = ads.computeVolts(adc1);
  volts2 = ads.computeVolts(adc2);
  volts3 = ads.computeVolts(adc3);

  tempCurrentRaw = (1000 * volts0) / 120;
  rhCurrentRaw = (1000 * volts1) / 120;
  pressureCurrentRaw = (1000 * volts2) / 120;
  current3 = (1000 * volts3) / 120;

  tempBuffer[bufferIndex] = analogScaling(tempCurrentRaw, TEMP_MIN_MA, TEMP_MAX_MA, TEMP_MIN_F, TEMP_MAX_F, TEMP_CAL_OFFSET);
  rhBuffer[bufferIndex] = analogScaling(rhCurrentRaw, RH_MIN_MA, RH_MAX_MA, RH_MIN, RH_MAX, RH_CAL_OFFSET);
  pressureBuffer[bufferIndex] = analogScaling(pressureCurrentRaw, PRESSURE_MIN_MA, PRESSURE_MAX_MA, PRESSURE_MIN_PSI, PRESSURE_MAX_PSI, PRESSURE_CAL_OFFSET);

  temperature = calculateAverage(tempBuffer, BUFFER_SIZE);
  humidity = calculateAverage(rhBuffer, BUFFER_SIZE);
  pressure = calculateAverage(pressureBuffer, BUFFER_SIZE);

  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
}

void readAM2315()
{
  if (!am2315.readTemperatureAndHumidity(&temperature, &humidity))
  {
    Serial.println("Failed to read data from AM2315");
    return;
  }
}

void updateRelay()
{
  unsigned long currentTime = millis();

  // Check if the blower fan is on and if enough time has passed since the last toggle
  if (digitalRead(BLOWER_FAN_PIN) == HIGH && currentTime - lastRelayToggleTime >= relayToggleDelay)
  {
    float lowerLimit = humiditySetpoint - deadband;
    float upperLimit = humiditySetpoint + deadband;

    // Turn relay ON if humidity is below lower limit
    if (!relayState && humidity < lowerLimit)
    {
      relayState = true;
      digitalWrite(HUMIDIFIER_RELAY_PIN, HIGH);
      lastRelayToggleTime = currentTime;
      Serial.println("Relay On");
    }

    // Turn relay OFF if humidity is above upper limit
    else if (relayState && humidity > upperLimit)
    {
      relayState = false;
      digitalWrite(HUMIDIFIER_RELAY_PIN, LOW);
      lastRelayToggleTime = currentTime;
      Serial.println("Relay Off");
    }
  }
}

void updateDisplay()
{
  if (rawRead)
  {
    display.clearDisplay();
    display.setTextSize(1.5);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.print("Temp:");
    display.setCursor(60, 0);
    display.printf("%6.2f F", temperature);

    display.setCursor(0, 16);
    display.print("RH:");
    display.setCursor(60, 16);
    display.printf("%6.2f %%", humidity);

    display.setCursor(0, 32);
    display.print("Press:");
    display.setCursor(60, 32);
    display.printf("%6.2f psi", pressure);

    display.setCursor(0, 48);
    display.print("Relay:");
    display.setCursor(60, 48);
    display.print(relayState ? "ON" : "OFF");

    display.display();
  }
  else
  {
    display.clearDisplay();
    display.setTextSize(1.5);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.print("RawT:");
    display.setCursor(60, 0);
    display.printf("%6.2f mA", tempCurrentRaw);

    display.setCursor(0, 16);
    display.print("RawRH:");
    display.setCursor(60, 16);
    display.printf("%6.2f mA", rhCurrentRaw);

    display.setCursor(0, 32);
    display.print("RawP:");
    display.setCursor(60, 32);
    display.printf("%6.2f mA", pressureCurrentRaw);

    display.setCursor(0, 48);
    display.print("Relay:");
    display.setCursor(60, 48);
    display.print(relayState ? "  ON" : "  OFF");

    display.display();
  }
}

void serialDisplay()
{
  Serial.println("-----------------------------------------------------------");
  Serial.print("AIN0: ");
  Serial.print(adc0);
  Serial.print("  ");
  Serial.print(volts0, 4);
  Serial.print("mV");
  Serial.print("  ");
  Serial.print("T(mA): ");
  Serial.print(tempCurrentRaw);
  Serial.print("  ");
  Serial.print("T: ");
  Serial.println(temperature);

  Serial.print("AIN1: ");
  Serial.print(adc1);
  Serial.print("  ");
  Serial.print(volts1, 4);
  Serial.print("mV");
  Serial.print("  ");
  Serial.print("RH(mA): ");
  Serial.print(rhCurrentRaw);
  Serial.print("  ");
  Serial.print("RH: ");
  Serial.println(humidity);

  Serial.print("AIN2: ");
  Serial.print(adc2);
  Serial.print("  ");
  Serial.print(volts2, 4);
  Serial.print("mV");
  Serial.print("  ");
  Serial.print("RH(mA): ");
  Serial.print(pressureCurrentRaw);
  Serial.print("  ");
  Serial.print("RH: ");
  Serial.println(pressure);

  Serial.print("AIN3: ");
  Serial.print(adc3);
  Serial.print("  ");
  Serial.print(volts3);
  Serial.println("V");

  Serial.print("Temp *C: ");
  Serial.print(am2315Temperature);
  Serial.print("RH %: ");
  Serial.println(am2315humidity);
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

  // Sensors reading
  if (currentTime - lastSensorReadTime >= sensorReadInterval)
  {
    lastSensorReadTime = currentTime;
    readSensors();
    updateRelay();
  }

  // AM2315 reading
  if (currentTime - lastAM2315ReadTime >= AM2315ReadInterval)
  {
    lastAM2315ReadTime = currentTime;
    readAM2315();
  }

  // MQTT Broker
  // if (currentTime - lastMqttPublishTime >= mqttPublishInterval)
  // {
  //   lastMqttPublishTime = currentTime;
  //   publishToMQTT();
  // }

  // Display
  if (currentTime - lastDisplayUpdateTime >= displayUpdateInterval)
  {
    lastDisplayUpdateTime = currentTime;
    updateDisplay();
  }

  // Serial
  if (currentTime - lastSerialUpdateTime >= displayUpdateInterval)
  {
    lastSerialUpdateTime = currentTime;
    serialDisplay();
  }

  // Button state
  static bool lastState = rawRead;
  if (rawRead != lastState)
  {
    lastState = rawRead;
  }
}