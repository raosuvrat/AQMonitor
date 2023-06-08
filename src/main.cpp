#include <Arduino.h>
#include <ArduinoOTA.h>
#include <DHT.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <MD_MAX72xx.h>
#include <MD_Parola.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#include "Adafruit_CCS811.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#define WIFI_AP_SSID "SSID"
#define WIFI_AP_PASS "PASS"
#define MDNS_NAME "HOSTNAME"
#define MQTT_SERVER "10.0.0.10"
#define MQTT_USER "USER"
#define MQTT_PASS "PASS"
#define MQTT_PORT 1883
#endif

#define DHT_PIN D6
#define PMS_RX D3
#define PMS_TX D4
#define MD_MAX_CS D8
#define MD_MAX_DEVICES 10
#define MD_MAX_SPEED 40

MD_Parola P(MD_MAX72XX::FC16_HW, MD_MAX_CS, MD_MAX_DEVICES);
SoftwareSerial pmsSerial(PMS_RX, PMS_TX);

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um,
      particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;
ESP8266WebServer server(80);
DHT dht(DHT_PIN, DHT11);
WiFiClient espClient;
PubSubClient client(espClient);

char buf[2048] = {0};
Adafruit_CCS811 ccs;
String res = "";

bool readPMSdata(Stream *s) {
  if (!s->available()) {
    return false;
  }

  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }

  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  return true;
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_AP_SSID, WIFI_AP_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  MDNS.begin(MDNS_NAME);
  server.on("/", []() { server.send(200, "text/plain", res.c_str()); });
  server.begin();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP8266Client", MQTT_USER, MQTT_PASS)) {
    } else {
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(9600);
  client.setServer(MQTT_SERVER, MQTT_PORT);

  ArduinoOTA.begin();

  pinMode(DHT_PIN, INPUT);
  Wire.begin();
  ccs.begin();

  pmsSerial.begin(9600);
  initWiFi();
  P.begin();
  P.setCharSpacing(1);
  P.setTextAlignment(PA_CENTER);
  P.setIntensity(15);
  dht.begin();
  sprintf(buf, "Hello!");
  P.displayScroll(buf, PA_CENTER, PA_SCROLL_LEFT, MD_MAX_SPEED);
  ccs.setDriveMode(CCS811_DRIVE_MODE_1SEC);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  while (!P.displayAnimate()) {
    client.loop();
    ArduinoOTA.handle();
    server.handleClient();
  }

  if (readPMSdata(&Serial)) {
    int co, tvoc;
    double hum, temp;

    ccs.setEnvironmentalData(dht.readHumidity(), dht.readTemperature());
    ccs.readData();

    co = ccs.geteCO2();
    tvoc = ccs.getTVOC();
    hum = dht.readHumidity();
    temp = dht.readTemperature(true);

    sprintf(buf,
            // "0.3um:%d  0.5um:%d  1.0um:%d  2.5um:%d  5.0um:%d  10.0um:%d "
            "PM1.0: %d  PM2.5: %d  PM10.0: %d  "  //  PM1.0(E):%d  PM2."
            "Temp: %.1f  Hum: %.1f  "
            "C02: %d  TVOC: %d  ",
            // data.particles_03um, data.particles_05um, data.particles_10um,
            // data.particles_25um, data.particles_50um, data.particles_100um,
            data.pm10_standard, data.pm25_standard, data.pm100_standard,
            // data.pm10_env, data.pm25_env, data.pm100_env,
            temp, hum, co, tvoc);

    client.publish("sensor/temperature", String(temp).c_str(), false);
    client.publish("sensor/humidity", String(hum).c_str(), false);
    client.publish("sensor/co2", String(co).c_str(), false);
    client.publish("sensor/tvoc", String(tvoc).c_str(), false);
    client.publish("sensor/pm1", String(data.pm10_standard).c_str(), false);
    client.publish("sensor/pm2.5", String(data.pm25_standard).c_str(), false);
    client.publish("sensor/pm10", String(data.pm100_standard).c_str(), false);

    res = String(buf);
    P.displayScroll(buf, PA_CENTER, PA_SCROLL_LEFT, MD_MAX_SPEED);
  }
}