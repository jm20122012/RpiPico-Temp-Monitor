#include <DHT11.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* MQTT_BROKER = "";
const int MQTT_PORT = 1883;
const char* SSID = "=";
const char* PW = "";
const char* PUB_TOPIC = "dht11_sensors/downstairs";
const unsigned long PUB_INTERVAL = 30 * 1000; // In milliseconds
const unsigned long LED_BLINK_INTERVAL = 3 * 1000; // In milliseconds
const unsigned long WIFI_CHECK_INTERVAL  = 10 * 1000; // In milliseconds

// Global variables
bool ledState = false;
unsigned long lastLedBlink = 0;
unsigned long lastPublish = 0;
unsigned long lastWifiCheck = 0;

DHT11 dht11(15);
WiFiClient mqttClient;
PubSubClient client(mqttClient);

void setup() {

  Serial.begin(115200);
  
  connectWifi();

  // MQTT Setup
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setBufferSize(1024);

  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  if (!client.connected()) {
    mqttReconnect();
  }

  unsigned long currentTime = millis();
 
  if (currentTime - lastWifiCheck >= WIFI_CHECK_INTERVAL){
    connectWifi();
    lastWifiCheck = currentTime;
  }

  if (currentTime - lastPublish >= PUB_INTERVAL){
    int tempC = 0;
    int humidity = 0;
    int result = dht11.readTemperatureHumidity(tempC, humidity);

    if (result == 0){
      int tempF = celsiusToFahrenheit(tempC);
      Serial.println("------------------------------------------");
      Serial.print("Temp C: ");
      Serial.print(tempC);
      Serial.print(" | Temp F: ");
      Serial.print(tempF);
      Serial.print(" | Humidity: ");
      Serial.println(humidity);
      Serial.println("------------------------------------------");
      publishStatus(tempF, tempC, humidity);
    } else {
      Serial.println(DHT11::getErrorString(result));
    }
    lastPublish = currentTime;
  }

  if (currentTime - lastLedBlink >= LED_BLINK_INTERVAL){
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastLedBlink = currentTime;
  }
}

int celsiusToFahrenheit(int tempC) {
  return (tempC * 9 / 5) + 32;
}

void connectWifi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Attempting to connect...");
    WiFi.disconnect();
    WiFi.begin(SSID, PW);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
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
    Serial.println("WiFi connection is stable");
  }
}

void mqttReconnect() {
  int attempts = 0;

  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    String clientId = "dht11_sensor-" + String(random(0xffff), HEX);
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

void publishStatus(int tempF, int tempC, int humidity) {
  JsonDocument doc;

  doc["tempF"] = tempF;
  doc["tempC"] = tempC;
  doc["humidity"] = humidity;

  char jsonBuffer[128];  // Increased buffer size
  size_t jsonSize = serializeJson(doc, jsonBuffer);
  Serial.println("JSON payload size: " + String(jsonSize) + " bytes");

  if (!client.connected()) {
    Serial.println("MQTT client disconnected. Attempting to reconnect...");
    mqttReconnect();
  }

  // Serial.println("Attempting to publish. Client state: " + String(client.state()) + ", Connected: " + String(client.connected()));
  bool pubResult = client.publish(PUB_TOPIC, jsonBuffer);
  // Serial.println("Publish result: " + String(pubResult) + ", Client state after publish: " + String(client.state()));

  if (pubResult) {
    Serial.println("Status published successfully");
  } else {
    Serial.println("Failed to publish status. MQTT state: " + String(client.state()));
  }
}
