#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define SEALEVELPRESSURE_HPA (1013.25)

const char* MQTT_BROKER = "";
const int MQTT_PORT = 1883;
const char* SSID = "";
const char* PW = "";
const char* PUB_TOPIC = "bmp280_sensors/basement";
const unsigned long PUB_INTERVAL = 60 * 1000; // In milliseconds
const unsigned long LED_BLINK_INTERVAL = 3 * 1000; // In milliseconds
const unsigned long WIFI_CHECK_INTERVAL  = 10 * 1000; // In milliseconds
const unsigned long MQTT_CHECK_INTERVAL = 10 * 1000; // In milliseconds
const String CLIENT_ID = "bmp280_sensor_basement" + String(random(0xffff), HEX);
const char* BOARD_TYPE = "picow";

// Global variables
bool ledState = false;
unsigned long lastLedBlink = 0;
unsigned long lastPublish = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastMqttCheck = 0;

Adafruit_BMP280 bmp; // I2C (default pins for Raspberry Pi Pico: GPIO 4 (SDA), GPIO 5(SCL)
WiFiClient mqttClient;
PubSubClient client(mqttClient);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  blinkLed(10, 50);

  Serial.begin(9600);
  while (!Serial) {
    delay(100);
  }

  Serial.println("Starting setup...");
  
  blinkLed(5, 100);

  bool status = bmp.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  blinkLed(3, 100);

  connectWifi();

  // MQTT Setup
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setBufferSize(1024);


}

void loop() {

  unsigned long currentTime = millis();
 
  if (currentTime - lastWifiCheck >= WIFI_CHECK_INTERVAL){
    Serial.println("Checking Wifi connection...");
    connectWifi();
    lastWifiCheck = currentTime;
  }

  if (currentTime - lastMqttCheck >= MQTT_CHECK_INTERVAL){
    Serial.print("Checking MQTT connection...");
    bool mqttConnected = client.connected();
    
    if (mqttConnected){
      Serial.println("OK");
    } else {
      Serial.println("MQTT connection lost. Attempting to reconnect...");
      mqttReconnect();
    }

    lastMqttCheck = currentTime;
  }

  if (currentTime - lastPublish >= PUB_INTERVAL){
    Serial.println("Getting sensor readings...");
    
    float tempC = bmp.readTemperature();
    float tempF = celsiusToFahrenheit(tempC);
    // float humidity = bmp.readHumidity();
    float pressure = bmp.readPressure() / 100.0F;

    Serial.println("-----------------------------------------------------------------");
    Serial.print("Temp C: ");
    Serial.print(tempC);
    Serial.print(" | Temp F: ");
    Serial.print(tempF);
    // Serial.print(" | Humidity: ");
    // Serial.print(humidity);
    Serial.print(" | Pressure: ");
    Serial.println(pressure);
    Serial.println("-----------------------------------------------------------------");
    publishStatus(tempF, tempC, pressure);
    lastPublish = currentTime;
  }

  if (currentTime - lastLedBlink >= LED_BLINK_INTERVAL){
    // Serial.println("Toggling LED...");
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastLedBlink = currentTime;
  }
}

void blinkLed(int count, int delayMilliseconds){
  for (int i = 0; i < count; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delayMilliseconds);
    digitalWrite(LED_BUILTIN, LOW);
    delay(delayMilliseconds);
  }
}

float celsiusToFahrenheit(float celsius) {
  return (celsius * 9.0 / 5.0) + 32.0;
}

void connectWifi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Attempting to connect...");
    WiFi.disconnect();
    WiFi.begin(SSID, PW);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      Serial.println("Wifi connection attempt " + String(attempts));
      delay(500);
      attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected to WiFi");
      IPAddress ip = WiFi.localIP();
      char ipStr[16];
      snprintf(ipStr, sizeof(ipStr), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
      Serial.println("IP address: " + String(ipStr));
    } else {
      Serial.println("Failed to connect to WiFi");
    }
  } else {
    Serial.println("WiFi connection is OK");
  }
}

void mqttReconnect() {
  int attempts = 0;

  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    String clientId = CLIENT_ID;
    if (client.connect(clientId.c_str())) {
      Serial.println("MQTT Connected");
    } else {
      attempts++;
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
    if (attempts >= 5){
      return;
    }
  }
}

void publishStatus(float tempF, float tempC, float pressure) {
  JsonDocument doc;

  doc["microcontrollerType"] = BOARD_TYPE;
  doc["tempF"] = tempF;
  doc["tempC"] = tempC;
  // doc["humidity"] = humidity;
  doc["pressure"] = pressure;

  char jsonBuffer[128];  // Increased buffer size
  size_t jsonSize = serializeJson(doc, jsonBuffer);
  Serial.println("JSON payload size: " + String(jsonSize) + " bytes");

  if (!client.connected()) {
    Serial.println("MQTT client disconnected while attempting to publish. Attempting to reconnect...");
    mqttReconnect();
    return;
  }

  Serial.println("Attempting to publish. Client state: " + String(client.state()) + ", Connected: " + String(client.connected()));
  bool pubResult = client.publish(PUB_TOPIC, jsonBuffer);
  Serial.println("Publish result: " + String(pubResult) + ", Client state after publish: " + String(client.state()));

  if (pubResult) {
    Serial.println("Status published successfully");
  } else {
    Serial.println("Failed to publish status. MQTT state: " + String(client.state()));
  }
}
