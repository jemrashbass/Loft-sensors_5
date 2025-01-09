#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <Jem_credentials_Barachini.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_BME680.h>
#include <Adafruit_Sensor.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <BH1750.h>
//#include <ElegantOTA.h>


#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep for (in seconds) */

#define ACK_TOPIC "esp32/ack"

bool messageAcknowledged = false;
bool awakeDueToPin27 = false;
unsigned long wakeupTime = 0; // Stores the time when wake-up occurred
const unsigned long awakePeriod = 0.5 * 60 * 1000; // 30s awake in ms
bool suspendSleep = false; // suspend sleep for OTA update 
unsigned long suspendSleepTimeout = 60000; // length in milliseconds
unsigned long suspendSleepSetTime = 0; // When suspendSleep was last set
bool timeToSleep = true; 


//parameter for PWM A02YYM depth sensor
#define DISTANCE_SENSOR 26
#define PWM_SIGNAL 25
#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8 // We are using 8 bits for resolution
#define PWM_FREQUENCY 20 // 10Hz corresponds to a period of 100ms
#define DUTY_CYCLE 100 // 50% duty cycle. Max value for 8-bit resolution is 255
#define CONVERSION_FACTOR 57 // 57 microseconds per cm

/*
//define flow sensor
#define FLOW_SENSOR1_PIN 16

//define RCWL sensor
#define RCWL_sensor 34
int state = LOW;            // by default, no motion detected
int RCWL_detected = 0;
#define  LED1 23
*/

//define lightmeter 
BH1750 lightMeter;

//water level variables
float tank_depth = 50; // the height of the empty tank in cm

//Define temperature sensor
#define DS18B20_PIN 13

/*
// define PIR and LED
//const int motionSensorPin = 16; // (marked PIR on Board)
//const int LED1 = 4; // LED connected to digital pin 4

// define ADC input from battery
#define BATTERY_INPUT 32

// define ADC input from solar panel
#define SOLAR_INPUT 35
*/

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(DS18B20_PIN);

// Pass our oneWire reference to DallasTemperature library instance
DallasTemperature sensors(&oneWire);

//Setup BME680 sensor om I2C
Adafruit_BME680 bme; 

// Variables to hold sensor readings
float bmetemperature;
float humidity;
float pressure;
float gasResistance;


// Address of the sensor
//DeviceAddress sensor1 = { 0x28, 0xB3, 0x9A, 0x0C, 0x0, 0x0, 0x0, 0x58 };
DeviceAddress sensor1 = { 0x28, 0x29, 0x13, 0x0F, 0x12, 0x21, 0x1, 0xC8 };
//DeviceAddress sensor1 = { 0x28, 0xDE, 0x61, 0x0C, 0x0, 0x0, 0x0, 0x07 };


RTC_DATA_ATTR int bootCount = 0;

/* 
volatile int pulseCount1 = 0;
float flowRate1;
float totalVolumePerHour1 = 0;
float totalVolumePerDay1 = 0;


unsigned long currentTime;
unsigned long previousTime;
unsigned long interval = 5000; // interval to calculate flow rate (in milliseconds)

unsigned long resetHourTime = 0;
unsigned long resetDayTime = 0;

void ICACHE_RAM_ATTR pulseCounter1() {
  pulseCount1++;
}

// set detection of pulses on pin 27

volatile bool pulseDetected = false;  // variable to keep track of pulse detection

void ICACHE_RAM_ATTR pulseDetection() {  // ISR to set pulseDetected to true when a pulse is detected
  pulseDetected = true;
}
*/

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

uint16_t lastPacketId = 0;

/*
AsyncWebServer server(80);  // Initialize the web server

unsigned long ota_progress_millis = 0;

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}
*/

void getBME680Readings(){
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  bmetemperature = bme.temperature;
  pressure = bme.pressure / 100.0;
  humidity = bme.humidity;
  gasResistance = bme.gas_resistance / 1000.0;
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");

  // define the topic to subscribe
  uint16_t packetIdSub1 = mqttClient.subscribe("esp32/update", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub1);
  //mqttClient.publish("esp32/update" 0, true, esp32/update "received an MQTT message");
  //Serial.println("Publishing at QoS 0");

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

void onMqttPublish(uint16_t packetId) {
  if (packetId == lastPacketId) {
    messageAcknowledged = true;
  }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  String messageTemp;
  for (int i = 0; i < len; i++) {
    messageTemp += (char)payload[i];
  }
  Serial.println(messageTemp);
  if (strcmp(topic, "esp32/update") == 0) {
    if (messageTemp == "start") {
      suspendSleep = true;
      suspendSleepSetTime = millis(); // Capture the time when suspendSleep was set
      Serial.println("Update start message received");

      // Acknowledge receipt of the start message
      mqttClient.publish(ACK_TOPIC, 0, false, "stop received");
    } else if (messageTemp == "stop") {
      suspendSleep = false;
      Serial.println("Update stop message received");

      // Acknowledge receipt of the stop message
      mqttClient.publish(ACK_TOPIC, 0, false, "stop received");
    }
  }
}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    connectToMqtt(); 
  }
}

void go_to_sleep() {
    Serial.println("Going to sleep now.");
    delay(1000);
    Serial.flush();
    esp_deep_sleep_start();
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); 

  bootCount++;
  Serial.printf("Boot number: %d\n", bootCount);
  
  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_27,1);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");


// Check the wake-up reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    awakeDueToPin27 = true;
    wakeupTime = millis(); // Capture the wake-up time
  }

 

  // setup PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_SIGNAL, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, DUTY_CYCLE);
  pinMode(DISTANCE_SENSOR, INPUT);
  /*
  pinMode(FLOW_SENSOR1_PIN, INPUT_PULLUP);
  attachInterrupt(FLOW_SENSOR1_PIN, pulseCounter1, FALLING);
  pinMode(RCWL_sensor, INPUT);
  pinMode(SOLAR_INPUT, INPUT);
  pinMode(BATTERY_INPUT, INPUT);
  pinMode(LED1, OUTPUT);
 

  previousTime = millis();
*/
  sensors.begin();
  sensors.setResolution(sensor1, 12);

 
  Wire.begin();

  lightMeter.begin();
  Serial.println(F("BH1750 Test begin"));

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }



  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);


  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_ID, MQTT_PASSWORD);
  connectToWifi();

  // Wait for WiFi to connect
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Waiting for WiFi...");
  }
// subscribe to MQTT topic to allow stay awake
  mqttClient.subscribe("esp32/update", 2);


// Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms


/*
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is the ElegantOTA site /update.");
  });

  ElegantOTA.begin(&server);    // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);
    // Disable Auto Reboot
  ElegantOTA.setAutoReboot(false);

  server.begin();
  Serial.println("HTTP server started");
*/
}


void loop() {
//ElegantOTA.loop();

// to pause for message reads from setup
delay(1000);
  {
/*
  currentTime = millis();
  
  // Reset accumulated volume per hour
  if (currentTime - resetHourTime >= 3600000) {
    resetHourTime = currentTime;
    totalVolumePerHour1 = 0;
  }

  // Reset accumulated volume per day
  if (currentTime - resetDayTime >= 86400000) {
    resetDayTime = currentTime;
    totalVolumePerDay1 = 0;
  }

  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;

    detachInterrupt(FLOW_SENSOR1_PIN);

    flowRate1 = (pulseCount1 / 450.0) * (60000.0 / interval); // Calculate flow rate in L/min


float volumePerInterval1 = (pulseCount1 / 450.0) * (interval / 1000.0);
totalVolumePerHour1 += volumePerInterval1;
totalVolumePerDay1 += volumePerInterval1;

*/

// Request temperature from DS18B20
  sensors.requestTemperatures();
  float temperature_water = sensors.getTempC(sensor1);  // Get the temperature in Celsius from the sensor at address sensor1

//get data from lightmeter
float lux = lightMeter.readLightLevel();
    Serial.print("Light: ");
    Serial.print(lux);
    Serial.println(" lx");

//get data from BME680
    getBME680Readings();
    Serial.println();
    Serial.printf("Temperature = %.2f ºC \n", bmetemperature);
    Serial.printf("Humidity = %.2f % \n", humidity);
    Serial.printf("Pressure = %.2f hPa \n", pressure);
    Serial.printf("Gas Resistance = %.2f KOhm \n", gasResistance);

/*
//read voltage on potentiometer
int solar_reading = analogRead(SOLAR_INPUT);
float solar_percentage = (solar_reading / 4095.0) * 100; // Scale to percentage

Serial.print("Solar Volts value: ");
Serial.println(solar_reading);
Serial.print("Solar volts percentage: ");
Serial.println(solar_percentage);


//read voltage on battery
int battery_reading = analogRead(BATTERY_INPUT);
float battery_percentage = (battery_reading / 4095.0) * 100; // Scale to percentage

Serial.print("Battery Volts value: ");
Serial.println(battery_reading);
Serial.print("Battery volts percentage: ");
Serial.println(battery_percentage);

 collect data from PIR
  int motionDetected = digitalRead(motionSensorPin);
  int RCWL_detected = digitalRead(RCWL_sensor);   // read sensor value

// If either sensor is triggered, turn the LED on
  if (motionDetected == HIGH || RCWL_detected == HIGH) {
    digitalWrite(LED1, HIGH);
  } else {
    // Otherwise, turn the LED off
    digitalWrite(LED1, LOW);
  }



// Print motion sensor data to serial monitor
  Serial.println(motionDetected);
*/

// collect data from distance sensor A02YY
  unsigned long duration = pulseIn(DISTANCE_SENSOR, HIGH);
  float distance = duration / CONVERSION_FACTOR; // Convert the pulse duration to distance in cm

  Serial.print("Duration of pulse in microseconds: ");
  Serial.println(duration);
  Serial.print("Distance in cm: ");
  Serial.println(distance);

  // Calculate the percentage of tank capacity
  float waterDepth = tank_depth - distance; // Depth of water in cm
  float capacityPercentage = (waterDepth / tank_depth) * 100; // Percentage of tank capacity

  Serial.print("Water level as percentage of tank capacity: ");
  Serial.println(capacityPercentage);

  delay(200);
  
  if (mqttClient.connected()) {
    DynamicJsonDocument jsonDoc(1024);
    JsonObject root = jsonDoc.to<JsonObject>();
    
    root["device_name"] = DEVICE_NAME;
    //root["sleep time in seconds"] = TIME_TO_SLEEP;
    root["ip_address"] = WiFi.localIP().toString();
    root["rssi"] = WiFi.RSSI();
  


  JsonObject device_data = root.createNestedObject("sensors");
    //device_data["flowRate1"] = flowRate1;
    //device_data["totalVolumePerHour1"] = totalVolumePerHour1;
    //device_data["totalVolumePerDay1"] = totalVolumePerDay1;
    device_data["temperature_water"] = temperature_water;
    device_data["pulse length in uS:"] = duration;
    device_data["distance in cm:"] = distance;
    device_data["water_level_percentage"] = capacityPercentage;
    //device_data["PIR"] = motionDetected;
    //device_data["RCWL-0516"] = RCWL_detected;
    device_data["Lightmeter"] = lux;
    //device_data["battery value"] = battery_reading;
    //device_data["battery value percentage"] = battery_percentage;
    //device_data["solar value"] = solar_reading;
    //device_data["solar value percentage"] = solar_percentage;
    device_data["bme680_temperature"] = bmetemperature;
    device_data["bme680_humidity"] = humidity;
    device_data["bme680_pressure"] = pressure;
    device_data["bme680_gas_resistance"] = gasResistance;


    String payload;
    serializeJson(root, payload);

    mqttClient.publish(MQTT_TOPIC_PREFIX, 0, false, payload.c_str());
    Serial.printf("Published JSON: %s\n", payload.c_str());

    // Wait for message acknowledgment for a maximum of 5 seconds
      unsigned long startMillis = millis();
        while (!messageAcknowledged && (millis() - startMillis <= 5000)) {
          yield(); // Allows background tasks to process, e.g., WiFi and MQTT
          }

        if (messageAcknowledged) {
            Serial.println("Message acknowledged by the broker.");
            messageAcknowledged = false; // Reset acknowledgment flag
        } else {
            Serial.println("Message acknowledgment timeout.");
            }
  }
/*

Serial.print("Flow Rate Sensor 1: ");
Serial.print(flowRate1);
Serial.print(" L/min | Total Volume Per Hour: ");
Serial.print(totalVolumePerHour1);
Serial.print(" L | Total Volume Per Day: ");
Serial.print(totalVolumePerDay1);
Serial.println(" L");

Serial.println();

pulseCount1 = 0;

attachInterrupt(FLOW_SENSOR1_PIN, pulseCounter1, RISING);
// MQTT message handling for suspend sleep
    if (suspendSleep) {
        if ((millis() - suspendSleepSetTime >= suspendSleepTimeout)) {
            suspendSleep = false;
            Serial.println("suspendSleep timeout elapsed, resuming normal operation.");
        } else {
            Serial.println("Sleep suspended due to MQTT message, staying awake.");
        }
    }

    // Awake due to pin 27 handling
    if (awakeDueToPin27) {
        if ((millis() - wakeupTime) <= awakePeriod) {
            Serial.println("Awake due to pin 27, within awake period.");
        } else {
            awakeDueToPin27 = false; // Reset flag as the awake period has elapsed
            Serial.println("Awake period elapsed, can go to sleep now.");
        }
    }
*/
    // Final decision to go to sleep
    if (!suspendSleep && !awakeDueToPin27) {
        Serial.println("Conditions for staying awake not met, going to sleep.");
        go_to_sleep();
    }

    delay(500); // Minor delay for loop stability
}
  }
