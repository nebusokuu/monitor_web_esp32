#include <Arduino.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

const int LED = 2;
const uint8_t _SDA = 26;
const uint8_t _SCK = 25;

Adafruit_AM2320 am2320 = Adafruit_AM2320();

// Pub/Sub
const char* mqttHost = "xxx.xxx.xxx.xxx"; // MQTTのIPかホスト名
const int mqttPort = 9999;       // MQTTのポート
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

const char* topic = "esp32";     // 送信先のトピック名
char* payload;                   // 送信するデータ

void connectWiFi();
void connectMqtt();

void setup() {
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    Serial.begin(115200);
    while (!Serial)
    {
      delay(10); // hang out until serial port opens
    }

    Serial.println("ESP32 Setup\r\n");

    Wire.begin(_SDA, _SCK);
    am2320.begin();

    // Connect WiFi
    connectWiFi();

    // Connect MQTT
    connectMqtt();
}

void loop() {
    float temp_f = am2320.readTemperature();
    float hum_f = am2320.readHumidity();

    int16_t temp_i = (float)temp_f * 10.0;
    int16_t hum_i = (float)hum_f * 10.0;

    char buff[100] = { 0 };
    payload = buff;

    sprintf(buff, "%4d %4d", temp_i, hum_i);

    // 送信処理 topic, payloadは適宜
    // payload = "payload";
    mqttClient.publish(topic, payload);
    delay(10000);

    // WiFi
    if ( WiFi.status() == WL_DISCONNECTED ) {
        connectWiFi(); 
    }
    // MQTT
    if ( ! mqttClient.connected() ) {
        connectMqtt();
    }
    mqttClient.loop();  

    // Serial.print("Temp: ");
    // Serial.println(am2320.readTemperature());
    // Serial.print("Hum: ");
    // Serial.println(am2320.readHumidity());

}

// WiFi
const char ssid[] = "xxxxxxxxx";
const char passwd[] = "xxxxxxxxx";

/**
 * Connect WiFi
 */
void connectWiFi()
{
    WiFi.disconnect(true);
    WiFi.begin(ssid, passwd);
    Serial.print("WiFi connecting...");

    int cnt = 0;
    while (WiFi.status() != WL_CONNECTED) { 
        delay(100);
        cnt +=1 ;
        if (cnt>50) { ESP.restart(); }
        Serial.print(".");
    }
    Serial.print(" connected. ");
    Serial.println(WiFi.localIP());
}

/**
 * Connect MQTT
 */
void connectMqtt()
{
    mqttClient.setServer(mqttHost, mqttPort);
    while (!mqttClient.connected())
    {
        Serial.println("Connecting to MQTT...");
        String clientId = "ESP32-" + String(random(0xffff), HEX);
        if (mqttClient.connect(clientId.c_str()))
        {
          Serial.println("connected");
        }
        delay(1000);
        randomSeed(micros());
    }
}