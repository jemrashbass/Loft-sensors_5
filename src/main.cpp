#include <Arduino.h>
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
//#include <Wire.h>
//#include <SPI.h>
#include <Adafruit_Sensor.h>
//#include "Adafruit_BME680.h"
//#include <TelnetStream.h>
//#include "TelnetPrint.h"
#include <DHT.h>
#include <VL53L0X.h>
//#include <HCSR04.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>


//Water sensor
#define water_sensor 17
//const int SensorDataPin = 14;

//DHT22 temperature and humidity
#define DHTPIN 33
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

//for flow calculation
long currentMillisflow = 0;
long previousMillisflow = 0;
int intervalflow= 1000;
boolean ledState = LOW;
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;


//Define the Digital Input on the ESP for RCWL-0516
#define movement_signal 5
#define LedPin 18

//change assigned pins for VL53L0X - to match wiring on board used previously for SR04
#define I2C_SDA 16
#define I2C_SCL 19


//tap IR sensor
const int tap_digital_pin = 39;
const int tap_analog_pin = 36;

float tap_position_analogue;
bool tap_position_digital;



//Pump running microphone
int sound_input = 32; //Define the Digital Input on the Arduino for the sensor signal
int Sensor_State = 0;

//int movement_signal
int Mov_Sensor_State = 1;

//int water_sensor
int Water_Sensor_State = 1;

//time of flight VL53L0X
VL53L0X tofsensor;



//pressure sensor
//float V, P;
//const float  OffSet = 464 ;

//adc input pin 35
const int analoguePin = 35;
int analogueValue = 0;

unsigned long lastTime = 0;
unsigned long timerDelay = 3000;  // send readings timer

float humidity;
float temperature;

//initialise DHT22
DHT dht(DHTPIN, DHTTYPE);

#define SENSOR  23 // flow meter

/*/ Telnetstream settings
byte mac[] = { 0x38, 0xf9, 0xd3, 0x75, 0x67, 0x36 };
IPAddress myIP(10, 0, 1, 28);
*/

//local wifi network settings

/*/RPR location data
#define WIFI_SSID "10RPRMandJ"
#define WIFI_PASSWORD "Th1s_1s_0ur_Netw0rk"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 1, 125)
*/

//HDI location data
#define WIFI_SSID "HealthDataInsight"
#define WIFI_PASSWORD "qu1ckstartgu1de"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(10, 0, 1, 30)

// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

// MQTT Topics
#define MQTT_PUB_LOFT_DIST "esp/VL53L0X/loft/distancecm"
#define MQTT_PUB_LOFT_Tank_percent "esp/VL53L0X/loft/tank_percent"
#define MQTT_PUB_LOFT_WATER_PRES "esp/WPS/loft/pressure"
#define MQTT_PUB_LOFT_VIBRATION "esp/sw420/loft/vibration"
#define MQTT_PUB_LOFT_TEMP "esp/dht22/loft/temp"
#define MQTT_PUB_LOFT_HUM "esp/dht22/loft/humidity"
#define MQTT_PUB_LOFT_Flow_rate "esp/yf-sr201/loft/flow"
#define MQTT_PUB_LOFT_Output_volume  "esp/yf-sr201/loft/volume"
#define MQTT_PUB_LOFT_movement  "esp/rcwl-0516/loft/movement"
#define MQTT_PUB_LOFT_water  "esp/gws/loft/water"
#define MQTT_PUB_LOFT_tap_position_a  "esp/tcrt5000/loft/tap_on_a"
#define MQTT_PUB_LOFT_tap_position_d  "esp/tcrt5000/loft/tap_on_d"
#define MQTT_PUB_LOFT_pin35_analogue  "esp/pin35_analogue"


//water level variables (depth in cm)
float tank_fill;
float distance_to_water;
float tank_depth = 400;
float tank_fraction;


AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

long unsigned previousMillis = 0;   // Stores last time temperature was published
const long interval = 1000;        // Interval at which to publish sensor readings

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

void onMqttConnect(bool sessionPresent) {
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

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}



void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(sound_input, INPUT); //Set pin as input for microphone for motor sound
  pinMode (movement_signal, INPUT); // sets the pin for the motion detector
  pinMode (LedPin, OUTPUT);
  pinMode(water_sensor, INPUT);
  pinMode(tap_digital_pin,INPUT); //sets the digital pin input for the tap IR detector (note pin 36 does not accept digital)

  Wire.begin(I2C_SDA, I2C_SCL); // SDA (16), SCL (19) for VL530X - rather than defaults

  tofsensor.setTimeout(1000);
/*  if (!tofsensor.init())
  {
    Serial.println("Failed to detect and initialize time of flight sensor");
    while (1) {}
  }
*/

//DHT 22 Start
  Serial.println(F("DHTxx test!"));
  dht.begin();
  delay(2000);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials("jem", "Th1spassw0rd");
  connectToWifi();


  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillisflow = 0;

  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);

}


void loop() {

  int sensorValue = digitalRead(movement_signal);
  if (sensorValue == HIGH) {
    digitalWrite (LedPin, HIGH);
  }
  else {
    digitalWrite (LedPin, LOW);
    }



  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds)
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;

  if ((millis() - lastTime) > timerDelay) {


  //Analog pressure sensor
  //V = analogRead(33) * 5.00 / 1024;     //Sensor output voltage
  //P = (V - OffSet) * 400;             //Calculate water pressure


/*
  /V = analogRead(33);     //Sensor output voltage
  //P = (V - OffSet);             //Calculate water pressure


  Serial.print("Voltage:");
  Serial.print(V, 3);
  Serial.println("V");

  Serial.print(" Water Pressure:");
  Serial.print(P, 1);
  Serial.println(" KPa");
  Serial.println();
*/
    lastTime =millis();


//DHT22 temp and humidity readings
humidity = dht.readHumidity();
  // Read temperature as Celsius (the default)
temperature = dht.readTemperature();

 /* // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return; */


  Serial.print(F("Humidity: "));
  Serial.print(humidity);
  Serial.println ("%");
  Serial.print(F("Temperature: "));
  Serial.print(temperature);
  Serial.print(F("°C "));
  Serial.println("\n" );

Serial.print("Motion sensor: ");
  Mov_Sensor_State = digitalRead(movement_signal);
  if (Mov_Sensor_State == 1) {
    Serial.println("Motion detected");
    Serial.println("\n" );

  }
  else {
    Serial.println("All quiet");
    Serial.println("\n" );
  }

  {
  Serial.print("Pump running: ");
  Sensor_State = digitalRead(sound_input);
  if (Sensor_State == 1) {
    Serial.println("Sound of pump running");
    Serial.println("\n" );

  }
  else {
    Serial.println("Pump Off");
    Serial.println("\n" );
  }
}

//tap open sensor
{
  tap_position_analogue = analogRead(tap_analog_pin);
  tap_position_digital = digitalRead(tap_digital_pin);

Serial.print("Tap Reading Analogue= ");
  Serial.print(tap_position_analogue);
  Serial.print("\t Tap Reading Digital= ");
  Serial.println(tap_position_digital);

}
//analogue signal pin35

{
  // Reading potentiometer value
  analogueValue = analogRead(analoguePin);
  Serial.print("Analogue PIN35= ");
  Serial.println(analogueValue);
  delay(500);
}

//water sensor

  Serial.print("Water sensor: ");
  Water_Sensor_State = digitalRead(water_sensor);
  if (Water_Sensor_State == 1) {
    Serial.println("No Water- all dry");
    Serial.println("\n" );
  }
  else {
    Serial.println("WATER DETECTED");
    Serial.println("\n" );
  }

//VL53L0X senseor
Serial.print("Distance to water:" "\t");
  Serial.print(tofsensor.readRangeSingleMillimeters());
  Serial.print("mm");
  Serial.println("\n" );
  distance_to_water = (tofsensor.readRangeSingleMillimeters());



if (distance_to_water >=(8000)) {
  tank_fraction = 1;
  tank_fill= (1-tank_fraction);
}
else {
  tank_fraction = (distance_to_water / tank_depth);
  tank_fill = (1 - tank_fraction);
}

  Serial.print("tank fraction:" "\t");
  Serial.print(tank_fraction);
  Serial.println("\n" );

  Serial.print("tank fill: " "\t");
  Serial.print(tank_fill);
  Serial.println("\n" );


  currentMillisflow = millis();
   if (currentMillisflow - previousMillisflow > intervalflow) {

    pulse1Sec = pulseCount;
    pulseCount = 0;

    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1000.0 / (millis() - previousMillisflow)) * pulse1Sec) / calibrationFactor;
    previousMillisflow = millis();

    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / 60) * 1000;

    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;

    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate: ");
    Serial.print(int(flowRate));  // Print the integer part of the variable
    Serial.print("L/min");
    Serial.print("\t");       // Print tab space

    // Print the cumulative total of litres flowed since starting
    Serial.print("Output Liquid Quantity: ");
    Serial.print(totalMilliLitres);
    Serial.print("mL / ");
    Serial.print(totalMilliLitres / 1000);
    Serial.println("L");
    Serial.println("\n" );

    // Publish an MQTT message on topic esp/yf-sr201/loft/flow
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_LOFT_Flow_rate, 1, true, String(flowRate).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_LOFT_Flow_rate, packetIdPub1);
    Serial.printf("Message: %.2f \n", flowRate);

    // Publish an MQTT message on topic esp/yf-sr201/loft/volume
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_LOFT_Output_volume, 1, true, String(totalMilliLitres).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LOFT_Output_volume, packetIdPub2);
    Serial.printf("Message: %i \n", totalMilliLitres);

    //Publish an MQTT message on topic esp/rcwl-0516/loft/movement
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_LOFT_movement, 1, true, String(int(Mov_Sensor_State)).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LOFT_movement, packetIdPub3);
    Serial.printf("Message: %i \n", Mov_Sensor_State);


    //Publish an MQTT message on topic esp/VL53L0X/loft/distancecm
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_LOFT_DIST, 1, true, String(distance_to_water).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LOFT_DIST, packetIdPub4);
    Serial.printf("Message: %i \n", distance_to_water);

    //Publish an MQTT message on topic esp/VL53L0X/loft/tank_percent
    uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_LOFT_Tank_percent, 1, true, String(tank_fill).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LOFT_DIST, packetIdPub5);
    Serial.printf("Message: %i \n", tank_fill);

    //Publish an MQTT message on topic esp/SW420/loft/vibration
    uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_LOFT_VIBRATION, 1, true, String(int(Sensor_State)).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LOFT_VIBRATION, packetIdPub6);
    Serial.printf("Message: %i \n", Sensor_State);

    //Publish an MQTT message on topic esp/SW420/loft/vibration
    uint16_t packetIdPub7 = mqttClient.publish(MQTT_PUB_LOFT_water, 1, true, String(int(Water_Sensor_State)).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LOFT_water, packetIdPub6);
    Serial.printf("Message: %i \n", Water_Sensor_State);

    //Publish an MQTT message on topic esp/dht22/loft/temp
    uint16_t packetIdPub8 = mqttClient.publish(MQTT_PUB_LOFT_TEMP, 1, true, String(temperature).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_LOFT_TEMP, packetIdPub8);
    Serial.printf("Message: %.2f \n", temperature);

    // Publish an MQTT message on topic esp/dht22/loft/humidity
    uint16_t packetIdPub9 = mqttClient.publish(MQTT_PUB_LOFT_HUM, 1, true, String(humidity).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LOFT_HUM, packetIdPub9);
    Serial.printf("Message: %.2f \n", humidity);

    // Publish an MQTT message on topic esp/tcrt5000/loft/tap_on_a
    uint16_t packetIdPub10 = mqttClient.publish(MQTT_PUB_LOFT_tap_position_a, 1, true, String(tap_position_analogue).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LOFT_tap_position_a, packetIdPub10);
    Serial.printf("Message: %i \n", tap_position_analogue);

    // Publish an MQTT message on topic esp/tcrt5000/loft/tap_on_d
    uint16_t packetIdPub11 = mqttClient.publish(MQTT_PUB_LOFT_tap_position_d, 1, true, String(tap_position_digital).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LOFT_tap_position_d, packetIdPub11);
    Serial.printf("Message: %i \n", tap_position_digital);

// Publish an MQTT message on topic esp/pin35_analogue
    uint16_t packetIdPub12 = mqttClient.publish(MQTT_PUB_LOFT_pin35_analogue, 1, true, String (int(analogueValue)).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LOFT_pin35_analogue, packetIdPub12);
    Serial.printf("Message: %i \n", analogueValue);

    /*/Publish an MQTT message on topic esp/DS18B20/tempC
    uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_TEMP2C, 1, true, String(temperature_Celsius).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_TEMP2C, packetIdPub5);
    Serial.printf("Message: %.2f \n", temperature_Celsius);

    //Publish an MQTT message on topic esp/DS18B20/tempF
    uint16_t packetIdPub7 = mqttClient.publish(MQTT_PUB_TEMP2F, 1, true, String(temperature_Fahrenheit).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_TEMP2F, packetIdPub5);
    Serial.printf("Message: %.2f \n", temperature_Fahrenheit);

    // Publish an MQTT message on topic esp/bme680/loft/pressure
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_LOFT_PRES, 1, true, String(pressure).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LOFT_PRES, packetIdPub3);
    Serial.printf("Message: %.2f \n", pressure);

    // Publish an MQTT message on topic esp/bme680/loft/gas
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_LOFT_GAS, 1, true, String(gasResistance).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LOFT_GAS, packetIdPub4);
    Serial.printf("Message: %.2f \n", gasResistance);
*/


}

 delay(200);
}}

}
