#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* MQTT_BROKER = "";
const int MQTT_PORT = 1883;
const char* SSID = "";
const char* PW = "";
const char* PUB_TOPIC = "dust_sensors/livingRoom";
const String CLIENT_ID = PUB_TOPIC + String(random(0xffff), HEX);
const unsigned long PUB_INTERVAL = 1 * 1000; // In milliseconds
const unsigned long LED_BLINK_INTERVAL = 3 * 1000; // In milliseconds
const unsigned long WIFI_CHECK_INTERVAL  = 10 * 1000; // In milliseconds
const unsigned long MQTT_CHECK_INTERVAL = 10 * 1000; // In milliseconds
const char* BOARD_TYPE = "esp32-wroom";
const int LED_BUILTIN = 2;
const int DUST_SENSOR_LED = 21;
const int DUST_SENSOR_INPUT = 13;
const int SENSOR_SAMPLE_TIME = 280; // Microseconds
const int SENSOR_DELTA_TIME = 40; // Microseconds
const int SENSOR_SLEEP_TIME = 9680; // Microseconds

// Dust quality thresholds:
// 3000 + = Very Bad
// 1050-3000 = Bad
// 300-1050 = Ordinary
// 150-300 = Good
// 75-150 = Very Good
// 0-75 = Excellent
const int NUM_THRESHOLDS = 6;
const float THRESHOLDS[] = {75.0, 150.0, 300.0, 1050.0, 3000.0, 3000.1};
const char* QUALITY_LABELS[] = {"EXCELLENT", "VERY GOOD", "GOOD", "ORDINARY", "BAD", "VERY BAD"};

// int state() return codes:
// -4 : MQTT_CONNECTION_TIMEOUT - the server didn't respond within the keepalive time
// -3 : MQTT_CONNECTION_LOST - the network connection was broken
// -2 : MQTT_CONNECT_FAILED - the network connection failed
// -1 : MQTT_DISCONNECTED - the client is disconnected cleanly
// 0 : MQTT_CONNECTED - the client is connected
// 1 : MQTT_CONNECT_BAD_PROTOCOL - the server doesn't support the requested version of MQTT
// 2 : MQTT_CONNECT_BAD_CLIENT_ID - the server rejected the client identifier
// 3 : MQTT_CONNECT_UNAVAILABLE - the server was unable to accept the connection
// 4 : MQTT_CONNECT_BAD_CREDENTIALS - the username/password were rejected
// 5 : MQTT_CONNECT_UNAUTHORIZED - the client was not authorized to connect
const int MQTT_STATE_COUNT = 10;
const int MQTT_CONNECTION_STATE_CODES[] = {-4, -3, -2, -1, 0, 1, 2, 3, 4, 5};
const char* MQTT_STATE_MSG[] = {"MQTT_CONNECTION_TIMEOUT", "MQTT_CONNECTION_LOST", "MQTT_CONNECT_FAILED", "MQTT_DISCONNECTED", "MQTT_CONNECTED", "MQTT_CONNECT_BAD_PROTOCOL", "MQTT_CONNECT_BAD_CLIENT_ID", "MQTT_CONNECT_UNAVAILABLE", "MQTT_CONNECT_BAD_CREDENTIALS", "MQTT_CONNECT_UNAUTHORIZED"};

float reading = 0; 
float rawReading = 0.0;
float calculatedVoltage = 0.0;
float dustDensity = 0.0;
const char* quality = "";
bool ledState = false;
unsigned long lastLedBlink = 0;
unsigned long lastPublish = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastMqttCheck = 0;

WiFiClient mqttClient;
PubSubClient client(mqttClient);

void setup() {
  Serial.begin(9600);
  Serial.println("Setting up...");

  Serial.print("Configuring LED_BUILTIN pin...");
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("OK");

  Serial.print("Testing LED_BUILTIN...");
  blinkLed(LED_BUILTIN, 100, 5);
  Serial.println("OK");
  
  Serial.print("Configuring DUST_SENSOR_LED pin...");
  pinMode(DUST_SENSOR_LED, OUTPUT);
  Serial.println("OK");

  Serial.print("Configuring DUST_SENSOR_INPUT pin...");
  pinMode(DUST_SENSOR_INPUT, INPUT);
  Serial.println("OK");

  connectWifi();

  Serial.print("Configuring MQTT...");
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setBufferSize(1024);
  Serial.println("OK");
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
    reading = getSensorReading();
    quality = parseQuality(reading);

    if (reading>36.455){
      Serial.println("Sensor reading: " + String(reading) + "ug/m3 | Quality: " + String(quality));
      publishStatus(reading, quality);
    } else {
      Serial.println("Sensor reading outside spec: " + String(reading));
      publishStatus(-999.0, "N/A");
    }
    lastPublish = currentTime;
  }

  if (currentTime - lastLedBlink >= LED_BLINK_INTERVAL){
    // Serial.println("Toggling LED...");
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastLedBlink = currentTime;
  }
}

void blinkLed(int ledPin, int delayMilliseconds, int count){
  for (int i = 0; i < count; i++){
    digitalWrite(ledPin, 1);
    delay(delayMilliseconds);
    digitalWrite(ledPin, 0);
    delay(delayMilliseconds);
  }
}

float getSensorReading(){
  digitalWrite(DUST_SENSOR_LED, LOW);
  delayMicroseconds(SENSOR_SAMPLE_TIME);
  rawReading = analogRead(DUST_SENSOR_INPUT);
  delayMicroseconds(SENSOR_DELTA_TIME); 
  digitalWrite(DUST_SENSOR_LED, HIGH);
  delayMicroseconds(SENSOR_SLEEP_TIME);

  calculatedVoltage = rawReading * (5.0 / 1024.0);
  dustDensity = 170 * calculatedVoltage - 0.1;

  return dustDensity;
}

const char* parseQuality(float val) {
    for (int i = 0; i < NUM_THRESHOLDS; i++) {
        if (val <= THRESHOLDS[i]) {
            return QUALITY_LABELS[i];
        }
    }
    return "UNKNOWN";  // This should never happen if thresholds cover all ranges
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
  while (!client.connected() && attempts < 5) {
    Serial.print("Attempting MQTT connection...");
    String clientId = CLIENT_ID;
    Serial.print("Client ID: ");
    Serial.println(clientId);
    Serial.print("Broker: ");
    Serial.println(MQTT_BROKER);
    Serial.print("Port: ");
    Serial.println(MQTT_PORT);
    if (client.connect(clientId.c_str())) {
      Serial.println("MQTT Connected");
    } else {
      attempts++;
      int state = client.state();
      Serial.print("Connection failed. Return code: ");
      Serial.print(state);
      Serial.print(" | Code Msg: ");
      Serial.println(parseMqttStateCode(state));
      Serial.println("Trying again in 5 seconds...");
      delay(5000);
    }
  }
}

void publishStatus(float dustDensity, const char* quality) {
  JsonDocument doc;

  doc["microcontrollerType"] = BOARD_TYPE;
  doc["dustDensity"] = dustDensity;
  doc["dustQuality"] = quality;

  char jsonBuffer[128];  // Increased buffer size
  size_t jsonSize = serializeJson(doc, jsonBuffer);
  Serial.println("JSON payload size: " + String(jsonSize) + " bytes");

  if (!client.connected()) {
    Serial.println("MQTT client disconnected while attempting to publish. Attempting to reconnect...");
    mqttReconnect();
    return;
  }

  Serial.println("Attempting to publish. Client state: " + String(client.state()) + ", Connected: " + String(client.connected()));
  bool pubResult = client.publish(PUB_TOPIC, jsonBuffer, true);
  Serial.println("Publish result: " + String(pubResult) + ", Client state after publish: " + String(client.state()));

  if (pubResult) {
    Serial.println("Status published successfully");
  } else {
    Serial.println("Failed to publish status. MQTT state: " + String(client.state()));
  }
}

const char* parseMqttStateCode(int code){
  for (int i = 0; i < MQTT_STATE_COUNT; i++){
    if (code == MQTT_CONNECTION_STATE_CODES[i]){
      return MQTT_STATE_MSG[i];
    }
  }
  return "Unknown state code";
}
