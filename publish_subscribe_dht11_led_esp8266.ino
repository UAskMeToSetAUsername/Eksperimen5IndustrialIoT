/*********
  Rui Santos
  Complete instructions at https://RandomNerdTutorials.com/smart-home-ebook/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/

#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <DHT.h>

#define WIFI_SSID " "
#define WIFI_PASSWORD " "

// BROKER
#define MQTT_HOST IPAddress( , , , ) //MQTT BROKER IP ADDRESS
/*for example:
#define MQTT_HOST IPAddress(192, 168, 1, 106)*/
#define MQTT_PORT 1883
#define BROKER_USER "pi"
#define BROKER_PASS "raspberry"

//MQTT Topics
#define MQTT_PUB_TEMP "esp/dht11/temperature"
#define MQTT_PUB_HUM "esp/dht11/humidity"

//MQTT Subscribe Topics
#define MQTT_SUB_DIGITAL "esp/digital/#"

// DHT11
#define DHTPIN D1  // Pin yang terhubung ke DHT11
#define DHTTYPE DHT11  // Tipe sensor DHT

DHT dht(DHTPIN, DHTTYPE);

// Variables to hold sensor readings
float temp;
float temperature;
float hum;
float humidity;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time a message was published
const long interval = 10000;        // Interval at which to publish values

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  // Subscribe to topic MQTT_SUB_DIGITAL when it connects to the broker
  uint16_t packetIdSub1 = mqttClient.subscribe(MQTT_SUB_DIGITAL, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub1);

}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  // Do whatever you want when you receive a message

  // Save the message in a variable
  String receivedMessage;
  for (int i = 0; i < len; i++) {
   Serial.println((char)payload[i]);
   receivedMessage += (char)payload[i];
  }
  // Save topic in a String variable
  String receivedTopic = String(topic);
  Serial.println("Received Topic: ");
  Serial.print(receivedTopic);
  
  // Check which GPIO we want to control
  int stringLen = receivedTopic.length();
  // Get the index of the last slash
  int lastSlash = receivedTopic.lastIndexOf("/");
  // Get the GPIO number (it's after the last slash "/")
  // esp/digital/GPIO
  String gpio = receivedTopic.substring(lastSlash+1, stringLen);
  Serial.print("DIGITAL GPIO: ");
  Serial.println(gpio);
  Serial.println("STATE: ");
  Serial.println(receivedMessage);

  // Check if it is DIGITAL
  if (receivedTopic.indexOf("digital") > 0) {
    //Set the specified GPIO as output
    pinMode(gpio.toInt(), OUTPUT);
    //Control the GPIO
    if (receivedMessage == "true"){
      digitalWrite(gpio.toInt(), HIGH);
    }
    else{
      digitalWrite(gpio.toInt(), LOW);
    }
  }
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  dht.begin();

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(BROKER_USER, BROKER_PASS);

  connectToWifi();
}

void loop() {

  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;

    // New DHT11 sensor readings
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();

    temp = temperature - 13;
    hum = humidity + 35;

    // Publish an MQTT message on topic esp/dht11/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Message: %.2f \n", temp);

    // Publish an MQTT message on topic esp/dht11/humidity
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
    Serial.printf("Message: %.2f \n", hum);
  }
}
