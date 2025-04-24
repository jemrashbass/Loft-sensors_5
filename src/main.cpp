#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <Jem_credentials_Barachini.h>
//#include <Jem_credentials_HDI.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_BME680.h>
#include <Adafruit_Sensor.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <BH1750.h>
//#include <ElegantOTA.h>

/*
 * Loft Sensors Project
 * This ESP32 project monitors various sensors and reports data via MQTT:
 * - BH1750 light sensor
 * - BME680 environmental sensor (temperature, humidity, pressure, gas)
 * - DS18B20 water temperature sensor
 * - A02YYM PWM distance sensor for water tank level
 * 
 * The device uses deep sleep to conserve power and wakes up periodically
 * to take readings and send them via MQTT.
 */

// Deep sleep configuration
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300      /* Time ESP32 will go to sleep for (in seconds) */

// MQTT acknowledgment topic
#define ACK_TOPIC "esp32/ack"

// Variables for message handling and sleep control
bool messageAcknowledged = false;
bool awakeDueToPin27 = false;
unsigned long wakeupTime = 0; // Stores the time when wake-up occurred
const unsigned long awakePeriod = 0.5 * 60 * 1000; // 30s awake in ms
bool suspendSleep = false; // suspend sleep for OTA update 
unsigned long suspendSleepTimeout = 60000; // length in milliseconds
unsigned long suspendSleepSetTime = 0; // When suspendSleep was last set
bool timeToSleep = true; 

// Parameters for A02YYM depth sensor
#define DISTANCE_SENSOR 26  // Input pin (connect to sensor's output)
#define PWM_SIGNAL 25       // Output pin (may not be needed for some sensors)
#define PWM_CHANNEL 0
#define PWM_RESOLUTION 8    // 8-bit resolution
#define PWM_FREQUENCY 10    // 10Hz corresponds to a period of 100ms
#define DUTY_CYCLE 127      // 50% duty cycle. 127 for 8-bit resolution (0-255)
#define CONVERSION_FACTOR 55.44 // Derived from linear regression of actual measurements
#define SENSOR_READ_ATTEMPTS 10 // Increased number of pulse readings to average
#define PULSE_TIMEOUT 100000   // 100ms timeout for pulse reading
#define PULSE_INTERVAL 100     // Increased delay between pulse readings
#define MAX_SENSOR_INIT_TIME 1000 // Time to wait for sensor to initialize in ms

// Define lightmeter 
BH1750 lightMeter;

// Water level variables
float tank_depth = 50;              // Physical height of tank in cm
float min_pulse_width = 100;        // Minimum valid pulse width in microseconds
float max_pulse_width = 60000;      // Maximum valid pulse width in microseconds

// Calibration points for tank level based on real measurements
float distance_tank_empty = 60;     // Distance in cm when tank is 0% empty
float distance_tank_full = 5;       // Distance in cm when tank is 100% full
float pulse_offset = -35.78;        // Y-intercept from linear regression

// Define temperature sensor
#define DS18B20_PIN 13

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(DS18B20_PIN);

// Pass our oneWire reference to DallasTemperature library instance
DallasTemperature sensors(&oneWire);

// Setup BME680 sensor on I2C
Adafruit_BME680 bme; 

// Variables to hold sensor readings
float bmeTemperature;
float humidity;
float pressure;
float gasResistance;

// Address of the DS18B20 temperature sensor
//DeviceAddress sensor1 = { 0x28, 0x23, 0xB9, 0x51, 0x0, 0x0, 0x0, 0x1A };
DeviceAddress sensor1 = { 0x28, 0x29, 0x13, 0x0F, 0x12, 0x21, 0x1, 0xC8 };

// Boot count stored in RTC memory (persists during deep sleep)
RTC_DATA_ATTR int bootCount = 0;

// MQTT client and WiFi reconnection timers
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

uint16_t lastPacketId = 0;

/**
 * Reads the BME680 sensor and stores values in global variables
 */
void getBME680Readings() {
  // Tell BME680 to begin measurement
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin BME680 reading"));
    // Initialize values to zero when sensor fails
    bmeTemperature = 0;
    pressure = 0;
    humidity = 0;
    gasResistance = 0;
    return;
  }
  
  // Wait for the measurement to complete
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete BME680 reading"));
    // Initialize values to zero when sensor fails
    bmeTemperature = 0;
    pressure = 0;
    humidity = 0;
    gasResistance = 0;
    return;
  }
  
  // Store the readings in global variables
  bmeTemperature = bme.temperature;
  pressure = bme.pressure / 100.0;
  humidity = bme.humidity;
  gasResistance = bme.gas_resistance / 1000.0;
}

/**
 * Connect to WiFi network
 */
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

/**
 * Connect to MQTT broker
 */
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

/**
 * Callback for successful MQTT connection
 */
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");

  // Subscribe to topics
  uint16_t packetIdSub1 = mqttClient.subscribe("esp32/update", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub1);
  
  // Send a connection message
  mqttClient.publish("esp32/status", 0, false, "Device connected and ready");
}

/**
 * WiFi event handler
 */
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

/**
 * MQTT publish acknowledgment callback
 */
void onMqttPublish(uint16_t packetId) {
  if (packetId == lastPacketId) {
    messageAcknowledged = true;
    Serial.printf("Message with ID %d acknowledged by broker\n", packetId);
  }
}

/**
 * MQTT message received callback
 */
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
      mqttClient.publish(ACK_TOPIC, 0, false, "start received");
    } else if (messageTemp == "stop") {
      suspendSleep = false;
      Serial.println("Update stop message received");

      // Acknowledge receipt of the stop message
      mqttClient.publish(ACK_TOPIC, 0, false, "stop received");
    }
  }
}

/**
 * MQTT disconnection callback
 */
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

/**
 * Enter deep sleep mode
 */
void go_to_sleep() {
    Serial.println("Going to sleep now.");
    delay(1000);
    Serial.flush();
    esp_deep_sleep_start();
}

/**
 * Print the reason for wake-up
 */
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

/**
 * Measure the pulse width from the PWM distance sensor using direct pulseIn function
 * Returns the pulse duration in microseconds or 0 if no valid pulse is detected
 */
unsigned long measurePWMPulse() {
  // First method: Use Arduino's built-in pulseIn function with timeout
  unsigned long duration = pulseIn(DISTANCE_SENSOR, HIGH, PULSE_TIMEOUT);
  
  if (duration > 0) {
    return duration;
  }
  
  // If pulseIn failed, try a manual approach
  // Wait for the pulse to go HIGH
  unsigned long startWait = micros();
  while(digitalRead(DISTANCE_SENSOR) == LOW) {
    if(micros() - startWait > PULSE_TIMEOUT) {
      return 0; // Timeout - no pulse detected
    }
  }
  
  // Record the start time of the HIGH pulse
  unsigned long startTime = micros();
  
  // Wait for the pulse to go LOW
  while(digitalRead(DISTANCE_SENSOR) == HIGH) {
    if(micros() - startTime > PULSE_TIMEOUT) {
      return 0; // Timeout - pulse too long
    }
  }
  
  // Return the duration of the HIGH pulse
  return micros() - startTime;
}

/**
 * Get the distance reading from the PWM sensor with multiple attempts
 * Returns the distance in cm or -1 if no valid reading
 */
float getDistanceReading() {
  // Try the simplest approach first - direct pulseIn
  Serial.println("DIAGNOSTIC: Direct pulseIn measurement:");
  unsigned long directDuration = pulseIn(DISTANCE_SENSOR, HIGH, PULSE_TIMEOUT);
  Serial.printf("DIAGNOSTIC: Direct pulseIn result: %lu μs\n", directDuration);
  
  // Check pin state as diagnostic
  int pinState = digitalRead(DISTANCE_SENSOR);
  Serial.printf("DIAGNOSTIC: Current pin state: %d (HIGH=%d, LOW=%d)\n", 
                pinState, HIGH, LOW);
  
  // Try reading with different timeout and print raw pin readings
  Serial.println("DIAGNOSTIC: Raw pin reading test:");
  for(int i = 0; i < 10; i++) {
    Serial.printf("DIAGNOSTIC: Raw pin reading #%d: %d\n", i, digitalRead(DISTANCE_SENSOR));
    delay(10);
  }
  
  // Now proceed with multiple averaged readings
  unsigned long totalDuration = 0;
  int validReadings = 0;

  // Take multiple readings to improve accuracy
  Serial.println("Taking multiple distance sensor readings:");
  for(int i = 0; i < SENSOR_READ_ATTEMPTS; i++) {
    unsigned long duration = measurePWMPulse();
    
    // Print raw result
    Serial.printf("Pulse #%d raw: %lu μs\n", i+1, duration);
    
    // Only count valid pulse readings - filter out very short pulses
    if(duration > min_pulse_width && duration < max_pulse_width) {
      totalDuration += duration;
      validReadings++;
      Serial.printf("Valid pulse #%d: %lu μs\n", i+1, duration);
    } else {
      Serial.printf("Invalid pulse #%d: %lu μs (outside range %lu-%lu)\n", 
                   i+1, duration, min_pulse_width, max_pulse_width);
    }
    
    // Wait between readings
    delay(PULSE_INTERVAL);
  }

  // Calculate average duration if we have valid readings
  if(validReadings > 0) {
    float avgDuration = totalDuration / (float)validReadings;
    
    // Apply the calibrated formula: distance = (pulse - offset) / conversion_factor
    float distance = (avgDuration - pulse_offset) / CONVERSION_FACTOR;
    
    Serial.printf("Average pulse duration: %.2f μs\n", avgDuration);
    Serial.printf("Calculated distance: %.2f cm\n", distance);
    
    return distance;
  } else {
    Serial.println("No valid pulses detected");
    
    // As a fallback, try the direct measurement if it was valid
    if(directDuration > min_pulse_width && directDuration < max_pulse_width) {
      float distance = (directDuration - pulse_offset) / CONVERSION_FACTOR;
      Serial.printf("Using direct measurement as fallback: %.2f cm\n", distance);
      return distance;
    }
    
    return -1;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); 

  bootCount++;
  Serial.printf("Boot number: %d\n", bootCount);
  
  // Print the wakeup reason for ESP32
  print_wakeup_reason();

  // Enable wake-up sources
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 1);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

  // Check the wake-up reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    awakeDueToPin27 = true;
    wakeupTime = millis(); // Capture the wake-up time
  }

  // Setup distance sensor 
  pinMode(DISTANCE_SENSOR, INPUT); // Set distance sensor pin as input
  
  // Some A02YYM sensors may not need a PWM input, they just output a PWM signal
  // But we'll set it up just in case your sensor model requires it
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_SIGNAL, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, DUTY_CYCLE);
  
  // Give the sensor some time to initialize
  Serial.println("Waiting for distance sensor to initialize...");
  delay(MAX_SENSOR_INIT_TIME);

  // Initialize DS18B20 temperature sensor
  sensors.begin();
  sensors.setResolution(sensor1, 12); // Set to 12-bit resolution

  // Initialize I2C
  Wire.begin();

  // Initialize BH1750 light sensor
  if (lightMeter.begin()) {
    Serial.println(F("BH1750 light sensor initialized"));
  } else {
    Serial.println(F("Failed to initialize BH1750 light sensor"));
  }

  // Initialize BME680 sensor
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
  } else {
    Serial.println(F("BME680 sensor initialized"));
    
    // Set up oversampling and filter initialization for BME680
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }

  // Initialize MQTT and WiFi timers
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  // Register WiFi event handler
  WiFi.onEvent(WiFiEvent);

  // Set up MQTT callbacks
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);
  
  // Configure MQTT connection
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_ID, MQTT_PASSWORD);
  
  // Connect to WiFi
  connectToWifi();

  // Wait for WiFi to connect
  unsigned long wifiStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < 10000) {
    delay(500);
    Serial.println("Waiting for WiFi...");
  }

  // Subscribe to MQTT topic to allow stay awake
  if(WiFi.status() == WL_CONNECTED) {
    mqttClient.subscribe("esp32/update", 2);
  }
}

void loop() {
  // Wait a moment before taking readings
  delay(1000);
  
  // Check if we're connected to both WiFi and MQTT before proceeding
  if(WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    // Read temperature from DS18B20
    sensors.requestTemperatures();
    float temperature_water = sensors.getTempC(sensor1);
    
    // Check if the DS18B20 reading is valid (not -127°C)
    if (temperature_water == -127.0) {
      Serial.println("DS18B20 sensor error or not connected!");
      temperature_water = 0; // Set to 0 on error
    } else {
      // Print water temperature
      Serial.print("Water temperature: ");
      Serial.print(temperature_water);
      Serial.println(" °C");
    }
    
    // Get light level from BH1750
    float lux = lightMeter.readLightLevel();
    Serial.print("Light: ");
    Serial.print(lux);
    Serial.println(" lx");
    
    // Read environmental data from BME680
    getBME680Readings();
    Serial.printf("Air Temperature = %.2f °C\n", bmeTemperature);
    Serial.printf("Humidity = %.2f %%\n", humidity);
    Serial.printf("Pressure = %.2f hPa\n", pressure);
    Serial.printf("Gas Resistance = %.2f KOhm\n", gasResistance);
    
    // Get distance reading from PWM sensor
    float distance = getDistanceReading();
    
    // Calculate water level percentage using calibration points
    float capacityPercentage = 0;
    
    if(distance > 0) {
      // Constrain the distance to be within our calibrated range
      if(distance < distance_tank_full) {
        distance = distance_tank_full;
      }
      if(distance > distance_tank_empty) {
        distance = distance_tank_empty;
      }
      
      // Calculate the percentage based on the calibrated range
      capacityPercentage = ((distance_tank_empty - distance) / 
                          (distance_tank_empty - distance_tank_full)) * 100.0;
      
      // Ensure the percentage is within 0-100% range
      if(capacityPercentage < 0) capacityPercentage = 0;
      if(capacityPercentage > 100) capacityPercentage = 100;
      
    } else {
      // Invalid reading, default to 0%
      capacityPercentage = 0;
    }
    
    Serial.print("Water level as percentage of tank capacity: ");
    Serial.print(capacityPercentage);
    Serial.println("%");
    
    // Print the calibration details for reference
    Serial.printf("Tank calibration: %0.1f cm (empty/0%%) to %0.1f cm (full/100%%)\n", 
                 distance_tank_empty, distance_tank_full);
    
    // Create JSON document for MQTT message
    DynamicJsonDocument jsonDoc(1024);
    JsonObject root = jsonDoc.to<JsonObject>();
    
    root["device_name"] = DEVICE_NAME;
    root["boot_count"] = bootCount;
    root["ip_address"] = WiFi.localIP().toString();
    root["rssi"] = WiFi.RSSI();
    
    JsonObject device_data = root.createNestedObject("sensors");
    device_data["temperature_water"] = temperature_water;
    device_data["pulse_duration_us"] = (distance > 0) ? (distance * CONVERSION_FACTOR) : 0;
    device_data["distance_cm"] = distance;
    device_data["water_level_percentage"] = capacityPercentage;
    device_data["light_level_lux"] = lux;
    device_data["bme680_temperature"] = bmeTemperature;
    device_data["bme680_humidity"] = humidity;
    device_data["bme680_pressure"] = pressure;
    device_data["bme680_gas_resistance"] = gasResistance;
    
    // Serialize and publish the JSON message
    String payload;
    serializeJson(root, payload);
    
    lastPacketId = mqttClient.publish(MQTT_TOPIC_PREFIX, 0, false, payload.c_str());
    Serial.printf("Published JSON with ID %d: %s\n", lastPacketId, payload.c_str());
    
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
  } else {
    Serial.println("Not connected to WiFi or MQTT - skipping sensor readings");
  }
  
  // Handle sleep suspension and wake conditions
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
  
  // Final decision to go to sleep
  if (!suspendSleep && !awakeDueToPin27) {
    Serial.println("Conditions for staying awake not met, going to sleep.");
    go_to_sleep();
  }
  
  // Remove the alternative method now that the sensor is working
  
  // Add a delay before the next loop iteration
  delay(5000);
}