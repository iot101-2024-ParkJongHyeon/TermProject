import sys
import paho.mqtt.client as mqtt
import time

server = "192.168.0.14"

humidity_topic = "id/bathroom/sensor/evt/humidity"
temperature_topic = "id/bathroom/sensor/evt/temperature"
motor_topic = "id/bathroom/motor/ctrl" 

humidity_threshold = 36

def on_connect(client, userdata, flags, rc):
    print("Connected with RC : " + str(rc))

    client.subscribe(humidity_topic)
    client.subscribe(temperature_topic)

def on_message(client, userdata, msg):
    try:
        payload = float(msg.payload.decode('utf-8'))
        print(f'Received from {msg.topic:38} {payload: >5.1f}')

        if msg.topic == humidity_topic:
            if payload >= humidity_threshold:
                print("Humidity exceeded threshold. Sending motor ON command.")
                client.publish(motor_topic, "ON")
            else:
                print("Humidity below threshold. Sending motor OFF command.")
                client.publish(motor_topic, "OFF")
        
        elif msg.topic == temperature_topic:
            print("Temperature received but no specific action defined.")

    except ValueError:
        print(f"Invalid message on {msg.topic}: {msg.payload.decode('utf-8')}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print("Connecting to MQTT Broker...")
client.connect(server, 1883, 60)
client.loop_forever()
