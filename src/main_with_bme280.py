import network, json, time
import bme280
from umqtt.simple import MQTTClient
from machine import Pin, I2C

# WiFi Setup
WIFI_SSID = ""
WIFI_PW = ""

# MQTT Setup
MQTT_BROKER_IP = ""
MQTT_BROKER_PORT = 1883
CLIENT_ID = "basement_sensor"
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
    i2c = I2C(0, scl=machine.Pin(21), sda=machine.Pin(20))
    sensor = bme280.BME280(i2c=i2c)
    return sensor

def initialize_led():
    led = Pin(16, Pin.OUT)
    return led

def get_readings(sensor):
    temp, pressure, humidity = sensor.values
    temp_c = float(temp[:-1])
    temp_f = round((float(temp[:-1]) * (9/5)) + 32, 2)
    pressure = float(pressure[:-3])
    humidity = float(humidity[:-1])
    
    return (temp_c, temp_f, pressure, humidity)

def main():
    connect_wifi()
    mqtt_client = connect_mqtt()
    sensor = initialize_sensor()
    led = initialize_led()
    
    mqtt_timer = time.time()
    led_timer = time.time()
    
    while True:
        if (time.time() - mqtt_timer) >= 600:
            temp_c, temp_f, pressure, humidity = get_readings(sensor)
            mqtt_client.publish(PUB_TOPIC, json.dumps({"device_id": "basement", "temp_c": temp_c, "temp_f": temp_f, "humidity": humidity}))
            mqtt_timer = time.time()
        
        if (time.time() - led_timer) >= 15:
            for i in range(2):
                led.on()
                time.sleep(0.05)
                led.off()
                time.sleep(0.05)
                
            led_timer = time.time()
    
if __name__ == "__main__":
    main()
    
    
