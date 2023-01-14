import network, json, time, dht
from umqtt.simple import MQTTClient
from machine import Pin

# WiFi Setup
WIFI_SSID = ""
WIFI_PW = ""

# MQTT Setup
MQTT_BROKER_IP = ""
MQTT_BROKER_PORT = 1883
CLIENT_ID = "office_sensor"
PUB_TOPIC = b'temp_sensors'

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PW)
    
    count = 0
    print("Connecting to WiFi...")
    while count < 10:
        if wlan.status() < 0 or wlan.status() >= 3:
            break
        count += 1
        print("Waiting for network connection...")
        time.sleep(1)
        
    if wlan.status() != 3:
        raise RuntimeError('Network connection failed')
    else:
        print('WiFi Connected!')
        status = wlan.ifconfig()
        print(f"IP Address: {status[0]}")
        
def connect_mqtt():
    def reconnect_mqtt():
        time.sleep(5)
        machine.reset()
        
    print("Connecting to MQTT broker...")
    try:
        client = MQTTClient(CLIENT_ID, MQTT_BROKER_IP, keepalive=3600)
        client.connect()
    except Exception as e:
        print(f"Error connecting to MQTT broker: {str(e)}")
        print("Resetting device...")
        reconnect_mqtt()
    else:
        print("Connected to MQTT!")
        return client


def initialize_sensor():
    sensor = dht.DHT11(Pin(20))
    return sensor

def main():
    connect_wifi()
    mqtt_client = connect_mqtt()
    sensor = initialize_sensor()
    
    while True:
        sensor.measure()
        temp_c = sensor.temperature()
        temp_f = round((temp_c * (9/5)) + 32, 2)
        humidity = sensor.humidity()
        
        print(f"{30 * '-'} Sensor Readings: {30 * '-'}")
        print(f"Temperaure (C): {temp_c}")
        print(f"Temperaure (F): {temp_f}")
        print(f"Humidity: {humidity}")
        
        mqtt_client.publish(PUB_TOPIC, json.dumps({"device_id": "office_sensor", "temp_c": temp_c, "temp_f": temp_f, "humidity": humidity}))
        
        time.sleep(600)
    
if __name__ == "__main__":
    main()
    
    
