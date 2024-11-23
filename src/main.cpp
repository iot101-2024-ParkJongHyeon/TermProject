#include <Arduino.h>
#include <ConfigPortal32.h>
#include <PubSubClient.h>
#include <DHTesp.h>
#include <Wire.h>

char*               ssid_pfix = (char*)"mqtt_sensor_";
String              user_config_html = ""
    "<p><input type='text' name='broker' placeholder='MQTT Server'>";

char                mqttServer[100];
const int           mqttPort = 1883;
unsigned long       pubInterval = 5000;
unsigned long       lastPublished = - pubInterval;

#define             DHTPIN 15
#define             MOTOR_PIN 16

DHTesp              dht;
int                 dht_interval = 2000;
unsigned long       lastDHTReadMillis = 0;
float               humidity = 0;
float               temperature = 0;
char                buf_temp[50];
char                buf_humi[50];

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void pubStatus();
void readDHT22();
void handleMotorControl(char* topic, byte* payload, unsigned int length);

void callback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.printf("Received message: %s from topic: %s\n", message.c_str(), topic);

    if (String(topic) == "id/bathroom/motor/ctrl") {
        if (message == "ON") {
            digitalWrite(MOTOR_PIN, HIGH);
            Serial.println("Motor ON");
        } else if (message == "OFF") {
            digitalWrite(MOTOR_PIN, LOW);
            Serial.println("Motor OFF");
        }
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW);

    loadConfig();
    if (!cfg.containsKey("config") || strcmp((const char*)cfg["config"], "done")) {
        configDevice();
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin((const char*)cfg["ssid"], (const char*)cfg["w_pw"]);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    dht.setup(DHTPIN, DHTesp::DHT22);

    Serial.printf("\nIP address : "); Serial.println(WiFi.localIP());

    if (cfg.containsKey("broker")) {
        sprintf(mqttServer, (const char*)cfg["broker"]);
    }
    client.setServer(mqttServer, mqttPort);
    client.setCallback(callback);

    while (!client.connected()) {
        Serial.println("Connecting to MQTT...");
        if (client.connect("MQTTSensor")) {
            Serial.println("Connected to MQTT broker");
            client.subscribe("id/bathroom/motor/ctrl");
        } else {
            Serial.print("Failed with state "); Serial.println(client.state());
            delay(2000);
        }
    }
}

void loop() {
    client.loop();

    unsigned long currentMillis = millis();
    if (currentMillis - lastPublished >= pubInterval) {
        lastPublished = currentMillis;
        readDHT22();
        sprintf(buf_temp, "%.1f", temperature);
        sprintf(buf_humi, "%.1f", humidity);
        pubStatus();
    }
}

void pubStatus() {
    client.publish("id/bathroom/sensor/evt/temperature", buf_temp);
    client.publish("id/bathroom/sensor/evt/humidity", buf_humi);
}

void readDHT22() {
    unsigned long currentMillis = millis();
    if (currentMillis > lastDHTReadMillis + dht_interval) {
        lastDHTReadMillis = currentMillis;
        humidity = dht.getHumidity();
        temperature = dht.getTemperature();
    }
}
