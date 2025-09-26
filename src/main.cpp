#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <DHTesp.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ============================= OLED Display Parameters ============================= */
#define SCREEN_WIDTH 128   // OLED display width in pixels
#define SCREEN_HEIGHT 64   // OLED display height in pixels

// SSD1306 display connected via I2C (SDA, SCL pins)
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/* Shared variables for OLED display */
float g_temp = 0.0;
float g_humi = 0.0;
float g_soil = 0.0;
int   g_light = 0;
float g_water = 0.0;
int   g_rgbScale = 0;
int   g_angle = 0;

/* ============================= WiFi & MQTT Parameters ============================= */
static char* ssid = "Wokwi-GUEST";
static char* password = "";
static char* mqtt_server = "test.mosquitto.org";
static char* mqtt_username = "Mohammad";
static char* mqtt_password = "@Tohid1381";
static int   mqtt_port = 1883;

WiFiClient my_client;
PubSubClient client(my_client);

/* ============================= Mode Button ============================= */
#define mode_button 5
bool isAuto = false;              // Initial state: Manual Mode
unsigned long pressStartTime;

/* ============================= Sensors ============================= */
// DHT Sensor
const int DHT_PIN = 13;
DHTesp dhtSensor;
struct DHTresult {
  float temp;
  float humi;
};

// Soil Moisture Sensor (simulated via potentiometer in Wokwi)
#define ADC_Resolution 12
#define SIG 33

// Ultrasonic Sensor for Water Level
#define trigPin 32
#define echoPin 35
#define Sound_Speed 0.034       // cm/us
#define Water_Tank 400.0        // Tank height in cm
bool isLowWater = false;

// LDR Sensor (Light Intensity)
#define Analog_Out 39
#define ADC_LDR_Resolution 12
#define Vref 3.3
#define LUX_MAX 50000.0
#define LUX_MIN 0.0
const float GAMMA = 0.7;
const float RL10 = 50;           // Resistance at 10 lux (kΩ)

/* ============================= Actuators ============================= */
// Fan and Pump Relays
#define fan_signal 19
#define pump_signal 18
unsigned long startPump;

// RGB LED (growth light)
#define Red_Pin 23
#define Blue_Pin 2
#define Green_Pin 4
#define R_Fixed 200   // 65%
#define G_Fixed 40    // 10%
#define B_Fixed 120   // 35%
#define R_Freq 5000
#define G_Freq 5000
#define B_Freq 5000
#define RGB_REsolution 8
#define Target_LUX 20000   // Minimum required LUX for growth

// Servo Motor (for window control)
#define servo_pin 15
#define servo_channel 4
#define servo_resolution 12
#define servo_freq 50

/* ============================= Function Definitions ============================= */

// (1) WiFi Setup
void wifi_setup() {
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(50);
  }
  Serial.print("\nWiFi connected. IP: ");
  Serial.println(WiFi.localIP());
}

// (2) Read DHT sensor values
DHTresult readDHT() {
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  DHTresult result;
  result.temp = data.temperature;
  result.humi = data.humidity;
  return result;
}

// (3) MQTT Connection
void mqtt_reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT Connection...");
    String clientID = "ESP32Client-";
    clientID += String(random(0xFFFF, HEX));  // Random client ID

    if (client.connect(clientID.c_str())) {
      Serial.println("MQTT connected successfully.");
      client.publish("greenhouse/mqtt", "Greenhouse initialization");

      // Subscribe to control topics
      client.subscribe("greenhouse/control/light");
      client.subscribe("greenhouse/control/fan");
      client.subscribe("greenhouse/control/pump");
      client.subscribe("greenhouse/control/window");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(client.state());
      Serial.println(" — retrying in 3 seconds...");
      delay(3000);
    }
  }
}

// (4) Set RGB LED Intensity
void setIntensity(int intensity) {
  float scale = intensity / 100.0;
  uint8_t r = R_Fixed * scale;
  uint8_t g = G_Fixed * scale;
  uint8_t b = B_Fixed * scale;

  ledcWrite(0, r);
  ledcWrite(1, g);
  ledcWrite(2, b);
}

// (5) Move Servo to Desired Angle
void writeServo(int angle) {
  int duty_us = map(angle, 0, 180, 500, 2500); // 500–2500 μs pulse = 0–180°
  int duty_cycle = (int)((float)duty_us / 20000.0 * 4095); // 20 ms period
  ledcWrite(servo_channel, duty_cycle);
}

// (6) Handle Incoming MQTT Messages
void callback(char* topic, byte* payload, unsigned int length) {
  if (!isAuto) {   // Only in Manual Mode
    String incommingMsg;
    for (int i = 0; i < length; i++) {
      incommingMsg += char(payload[i]);
    }

    if (String(topic) == "greenhouse/control/light") {
      Serial.println("Light intensity: " + incommingMsg + "%");
      g_rgbScale = incommingMsg.toInt();
      setIntensity(g_rgbScale);
    }
    else if (String(topic) == "greenhouse/control/fan") {
      Serial.println("Fan control: " + incommingMsg);
      digitalWrite(fan_signal, incommingMsg == "ON" ? HIGH : LOW);
    }
    else if (String(topic) == "greenhouse/control/pump") {
      Serial.println("Pump control: " + incommingMsg);
      digitalWrite(pump_signal, incommingMsg == "ON" ? HIGH : LOW);
    }
    else if (String(topic) == "greenhouse/control/window") {
      Serial.println("Servo angle: " + incommingMsg);
      g_angle = incommingMsg.toInt();
      writeServo(g_angle);
    }
  }
}


/* ==================================== Tasks ===================================== */

void Sensor_Task(void *parameters)
{
  while (true)
  {
    // Reading DHT values
    DHTresult DHToutput = readDHT();
    g_temp = DHToutput.temp;
    String temp = String(g_temp,2); // DHT temperature output
    Serial.print("Temperature: ");
    Serial.println(temp);

    g_humi = DHToutput.humi;
    String humi = String(g_humi, 2); // DHT humidity output
    Serial.print("Humidity: ");
    Serial.println(humi);

    // Reading soil moisture value
    int value = analogRead(SIG);
    float soil_moisture = (float(value) / 4095) * 100; // a percentage of soil moisture
    g_soil = soil_moisture;
    String soil_mois = String(g_soil, 2);
    Serial.print("Soil Moisture Percentage: ");
    Serial.print(soil_mois);
    Serial.println("%");

    // Reading LDR value (light intensity percentage)
    int analog_value = analogRead(Analog_Out);
    float voltage = (analog_value / 4095.0) * Vref; // convert analog value to voltage 
    float resistance = 10000 * voltage / (Vref - voltage); // convert voltage to resistance (using 10k series resistor inside module)
    float lux = pow(RL10 * 1000 * pow(10, GAMMA) / resistance, (1.0 / GAMMA)); //convert resistance to lux (using gamma model)
    int percent = (lux - LUX_MIN) / (LUX_MAX - LUX_MIN) * 100;
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    g_light = percent;
    String percent_publish = String(g_light);
    Serial.print("Light Intensity: ");
    Serial.print(percent_publish);
    Serial.println("%");

    // Measuring water level with ultrasonic sensor
    digitalWrite(trigPin, LOW);
    ets_delay_us(2);
    digitalWrite(trigPin, HIGH);
    ets_delay_us(10);
    digitalWrite(trigPin, LOW); // emits a sound wave from trig pin
    long duration = pulseIn(echoPin, HIGH); // measures the duration of HIGH pulse from echo pin in Microseconds
    float distance = (Sound_Speed * duration) / 2.0; // measure the distance from water
    g_water = Water_Tank - distance;
    isLowWater = (g_water >= 0.0 && g_water < 10.0);

    String water_level = String(g_water, 2);
    Serial.print("Water Level: ");
    Serial.print(water_level);
    Serial.println(" cm");

    // Publish data of sensors to the proper topics
    client.publish("greenhouse/sensors/temperature", temp.c_str());
    client.publish("greenhouse/sensors/humidity", humi.c_str());
    client.publish("greenhouse/sensors/soilMoisture", soil_mois.c_str());
    client.publish("greenhouse/sensors/waterLevel", water_level.c_str());
    client.publish("greenhouse/sensors/light", percent_publish.c_str());
    client.publish("greenhouse/mode/status", isAuto ? "Auto" : "Manual");
    client.publish("greenhouse/status", isLowWater ? "Low Water" : "Running");

    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
  
}

void OLED_Task (void *parameters)
{
  while (true)
  {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    // Sensors
    oled.setCursor(0, 0);
    oled.print("T: ");
    oled.print(g_temp, 2);
    oled.print(" C , ");

    oled.print("H: ");
    oled.print(g_humi, 2);
    oled.println(" %");

    oled.setCursor(0, 10);
    oled.print("S: ");
    oled.print(g_soil, 2);
    oled.print(" % , ");

    oled.print("L: ");
    oled.print(g_light);
    oled.println(" %");

    oled.setCursor(0, 20);
    oled.print("W: ");
    oled.print(g_water, 2);
    oled.println(" cm");

    // Actuators
    oled.setCursor(0, 30);
    oled.print("P: ");
    oled.print(digitalRead(pump_signal) ? "ON , " : "OFF , ");

    oled.print("F: ");
    oled.println(digitalRead(fan_signal) ? "ON" : "OFF");

    oled.setCursor(0, 40);
    oled.print("RGB: ");
    oled.print(g_rgbScale);

    oled.print("% , S: ");
    oled.print(g_rgbScale);
    oled.println("°");

    oled.setCursor(0, 50);
    oled.print("Mode: ");
    oled.println(isAuto ? "Auto" : "Manual");

    oled.display();
    vTaskDelay(3000 / portTICK_PERIOD_MS); // update screen every 3 sec
  }
  
}

void Button_Task (void *parameters)
{
  while (true)
  {
    if (digitalRead(mode_button)) // a pull down button
    {
      vTaskDelay(100 / portTICK_PERIOD_MS); // Debouncing
      pressStartTime = millis();
      isAuto = !isAuto;

      while (digitalRead(mode_button)) // If holding the button
      {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (millis() - pressStartTime >= 3000)
        {
          Serial.println("Restarting ESP32 in 1 second...");
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          ESP.restart();
        }
      }
      
    }
  }
  
}

void Scenario_Task (void *parameters)
{
  while (true)
  {
    if (isAuto) // If it is in auto mode
    {
      if (g_temp > 30.0 || g_humi > 70.0)
      {
        digitalWrite(fan_signal, HIGH);
        Serial.println("[Auto Mode] Fan set to ON state");
      }
      else
      {
        digitalWrite(fan_signal, LOW);
        Serial.println("[Auto Mode] Fan set to OFF state");
      }

      

      if (g_light < 40) // g_light unit is in %. g_light < 40% -> g_light < 20000 LUX
      {
        Serial.println("[Auto Mode] Setting intensity of growth LED...");
        int current_lux = (g_light * 50000) / 100;
        int error = Target_LUX - current_lux; // current_lux < 20000
        int pwmValue = map(error, 0, Target_LUX, 0, 255);
        float brightness = (pwmValue * 100) / 255;
        setIntensity(int(brightness));
      }
      else{
        setIntensity(0); // No light
      }


      if (g_temp > 30.0 && g_humi > 50.0)
      {
        Serial.println("[Auto Mode] Opening wondow...");
        writeServo(90); // Opening window
      }
      else{
        writeServo(0); // closing window
      }
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  
}

void Pump_Task (void *parameters)
{
  while (true)
  {
    if (isAuto)
    {
      if (g_humi < 30.0)
      {
        if (!isLowWater) // If there is enough water
        {
          Serial.println("[Auto Mode] Turning pump on for 10 sec...");
          // startPump = millis();
          // delay(50);
          // while (millis() - startPump < 10000) // Turn on for 10 secs
          // {
          //   digitalWrite(pump_signal, HIGH);
          // }

          // digitalWrite(pump_signal, LOW); // Turn off after 10 secs
          digitalWrite(pump_signal, HIGH);
          vTaskDelay(10000 / portTICK_PERIOD_MS); // run pump for 10 seconds
          digitalWrite(pump_signal, LOW);
        }
      }
    }
  }
  
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Welcome to my smart greenhouse system!");

  if(!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 10);
  oled.println("Welcome to my smart greenhouse system!");
  oled.display();
  delay(3000);

  wifi_setup();

  // Mode Button
  pinMode(mode_button, INPUT_PULLDOWN);

  // MQTT Setup
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // DHT sensor setup
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);

  // PWM setup for RGB LED
  ledcSetup(0, R_Freq, RGB_REsolution);
  ledcAttachPin(Red_Pin, 0);
  ledcSetup(1, G_Freq, RGB_REsolution);
  ledcAttachPin(Green_Pin, 1);
  ledcSetup(2, B_Freq, RGB_REsolution);
  ledcAttachPin(Blue_Pin, 2);

  // PWM for servo motor
  ledcSetup(servo_channel, servo_freq, servo_resolution);
  ledcAttachPin(servo_pin, servo_channel);

  // Soil Moisture Setup
  analogReadResolution(ADC_Resolution);
  pinMode(SIG, INPUT);

  // LDR Sensor Module Setup
  analogReadResolution(ADC_LDR_Resolution);
  analogSetPinAttenuation(Analog_Out, ADC_11db); // full 0–3.3V range
  pinMode(Analog_Out, INPUT);

  // Ultrasonic Sensor Setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Pump and Fan relay signals
  pinMode(fan_signal, OUTPUT);
  pinMode(pump_signal, OUTPUT);

  // Tasks
  xTaskCreatePinnedToCore(
    Sensor_Task,
    "Sensor Task",
    10000,         // Stack Size
    NULL,          // Parameters
    1,             // Priority
    NULL,          // Handler
    0              // Core Number
  );
  
  xTaskCreatePinnedToCore(
    OLED_Task,
    "OLED_Task",
    6000,         // Stack Size
    NULL,          // Parameters
    1,             // Priority
    NULL,          // Handler
    1              // Core Number
  );

  xTaskCreatePinnedToCore(
    Button_Task,
    "Button_Task",
    5000,         // Stack Size
    NULL,          // Parameters
    1,             // Priority
    NULL,          // Handler
    0              // Core Number
  );

  xTaskCreatePinnedToCore(
    Scenario_Task,
    "Scenario_Task",
    15000,         // Stack Size
    NULL,          // Parameters
    1,             // Priority
    NULL,          // Handler
    1              // Core Number
  );

  xTaskCreatePinnedToCore(
    Pump_Task,
    "Pump_Task",
    10000,         // Stack Size
    NULL,          // Parameters
    1,             // Priority
    NULL,          // Handler
    0              // Core Number
  );
}


void loop() {
  // MQTT Connection
  if(!client.connected()){
    mqtt_reconnect();
  }
  client.loop();

  delay(500);
}

