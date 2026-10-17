/*
# MBusino: M-Bus --> MQTT-Gateway

MBusino shield (MBUSINO_SENSORS): M-Bus/OneWire/I²C --> MQTT-Gateway with a shield for ESP8266 D1 mini or ESP32 S2 mini, see shield.h
MBusino Nano (MBUSINO_ETH): M-Bus --> MQTT-Gateway via Ethernet (or Wifi), only M-Bus, see nano.h
MBUSINO_SLAVES 5: up to 5 M-Bus slaves, see mbus.h

https://github.com/Zeppelin500/MBusino/

## Licence
****************************************************
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version. This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
****************************************************
*/

#include <Arduino.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <MBusinoLib.h>  // Library for decode M-Bus
#include <MBusCom.h>  // Library M-Bus communication
#include <ArduinoJson.h>
#include <EEPROM.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#else
#include <WiFi.h>
#include <AsyncTCP.h>
#endif
#ifdef MBUSINO_ETH
#include <SPI.h>
#include <ETH.h>
#endif
#ifdef MBUSINO_SENSORS
#include <OneWire.h>            // Library for OneWire Bus
#include <DallasTemperature.h>  //Library for DS18B20 Sensors
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#endif

#define MBUSINO_VERSION "1.2.0"

#define MBUS_ADDRESS 254 // broadcast address, polled with one slave

// EEPROM flag constants
#define EEPROM_CREDENTIALS_OLD 500
#define EEPROM_CREDENTIALS_NEW 501

MBusinoLib payload(254);
#ifdef MBUSINO_ETH
NetworkClient ethClient;
WiFiClient wfClient;
PubSubClient client;
HardwareSerial MbusSerial(1);
MBusCom mbus(&MbusSerial,20,21);
#elif defined(ESP8266)
WiFiClient espClient;
PubSubClient client(espClient);
MBusCom mbus(&Serial);
#else
WiFiClient espClient;
PubSubClient client(espClient);
HardwareSerial MbusSerial(1);
MBusCom mbus(&MbusSerial,37,39);
#endif
DNSServer dnsServer;
AsyncWebServer server(80);

// stored in the EEPROM at address 100, the layout must not change
struct settings {
  char ssid[65];
  char password[65];
  char mbusinoName[31];
  char broker[65];
  uint16_t mqttPort;
  uint16_t extension;
  char mqttUser[65];
  char mqttPswrd[65];
  uint32_t sensorInterval;
  uint32_t mbusInterval;
#if MBUSINO_SLAVES > 1
  uint8_t mbusSlaves;
  uint8_t mbusAddress1;
  uint8_t mbusAddress2;
  uint8_t mbusAddress3;
  uint8_t mbusAddress4;
  uint8_t mbusAddress5;
#endif
  bool haAutodisc;
  bool telegramDebug;
#if MBUSINO_SLAVES > 1
} userData = {"SSID","Password","MBusino","192.168.1.8",1883,5,"mqttUser","mqttPassword",5000,120000,1,0xFE,0,0,0,0,true,false};
#else
} userData = {"SSID","Password","MBusino","192.168.1.8",1883,5,"mqttUser","mqttPassword",5000,120000,true,false};
#endif

// layout before version 1.0, only read to migrate the settings
struct oldSettings {
  char ssid[30];
  char password[30];
  char mbusinoName[11];
  char broker[20];
  uint16_t mqttPort;
  uint16_t extension;
  char mqttUser[30];
  char mqttPswrd[30];
  uint32_t sensorInterval;
  uint32_t mbusInterval;
#if MBUSINO_SLAVES > 1
  uint8_t mbusSlaves;
  uint8_t mbusAddress1;
  uint8_t mbusAddress2;
  uint8_t mbusAddress3;
  uint8_t mbusAddress4;
  uint8_t mbusAddress5;
#endif
  bool haAutodisc;
  bool telegramDebug;
} oldUserData;

bool apMode = false;
bool credentialsReceived = false;
bool waitForRestart = false;
bool wifiReconnect = false;
uint16_t conCounter = 0;
int Startadd = 0x13;  // Start address for decoding
uint8_t adMbusMessageCounter = 0; // Counter for autodiscouver mbus message.
uint32_t minFreeHeap = 0;

unsigned long timerDebug = 0;
unsigned long timerReconnect = 0;
unsigned long timerWifiReconnect = 0;
unsigned long timerReboot = 0;

uint8_t eeAddrCredentialsSaved = 32;
uint16_t credentialsSaved = 123;  // shows if EEPROM used befor for credentials

void heapCalc();
void reconnect();
void ledPulse(uint16_t interval);
void ledMqtt(bool connected);
void hwMqttReconnected();
void hwMqttSubscribe();
void hwMqttCallback(char* topic, byte* payload);

// M-Bus debug log for the web interface, see mbusLog.h
#define MBUS_LOG_OK 0
#define MBUS_LOG_NO_DATA 1
#define MBUS_LOG_BAD_FRAME 2
void mbusLogAdd(uint8_t status, uint8_t address, const uint8_t *data);
void mbusLogSetJson(uint8_t address, const char *json);
#if defined(ESP32)
void WiFiEvent(WiFiEvent_t event);
#endif

//outsourced program parts
#include "html.h"
#include "autodiscover.h"
#include "mbus.h"
#include "mbusLog.h"
#ifdef MBUSINO_ETH
#include "nano.h"
#else
#include "shield.h"
#endif
#include "mqtt.h"
#include "guiServer.h"
#if defined(ESP32)
#include "networkEvents.h"
#endif

void loadSettings(){
  EEPROM.get(eeAddrCredentialsSaved, credentialsSaved);
  if(credentialsSaved == EEPROM_CREDENTIALS_OLD){  // Old version before 1.0 (size of some variables has changed)
    EEPROM.get(100, oldUserData );
    strncpy(userData.ssid, oldUserData.ssid, sizeof(userData.ssid) - 1);
    strncpy(userData.password, oldUserData.password, sizeof(userData.password) - 1);
    strncpy(userData.mbusinoName, oldUserData.mbusinoName, sizeof(userData.mbusinoName) - 1);
    strncpy(userData.broker, oldUserData.broker, sizeof(userData.broker) - 1);
    userData.mqttPort = oldUserData.mqttPort;
    userData.extension = oldUserData.extension ;
    strncpy(userData.mqttUser, oldUserData.mqttUser, sizeof(userData.mqttUser) - 1);
    strncpy(userData.mqttPswrd, oldUserData.mqttPswrd, sizeof(userData.mqttPswrd) - 1);
    userData.sensorInterval= oldUserData.sensorInterval;
    userData.mbusInterval = oldUserData.mbusInterval;
#if MBUSINO_SLAVES > 1
    userData.mbusSlaves = oldUserData.mbusSlaves;
    userData.mbusAddress1 = oldUserData.mbusAddress1;
    userData.mbusAddress2 = oldUserData.mbusAddress2;
    userData.mbusAddress3 = oldUserData.mbusAddress3;
    userData.mbusAddress4 = oldUserData.mbusAddress4;
    userData.mbusAddress5 = oldUserData.mbusAddress5;
#endif
    userData.haAutodisc = oldUserData.haAutodisc;
    userData.telegramDebug = oldUserData.telegramDebug;

    EEPROM.put(100, userData);
    credentialsSaved = EEPROM_CREDENTIALS_NEW;
    EEPROM.put(eeAddrCredentialsSaved, credentialsSaved);
  }
  else if(credentialsSaved == EEPROM_CREDENTIALS_NEW){  // Version after 1.0 (size of some variables has changed)
    EEPROM.get(100, userData );
  }
  if(userData.telegramDebug > 1){
    userData.telegramDebug = 0;
  }
}

// fill the setup page with the stored settings, the order follows the fields in html.h
void buildSetupPage(){
  snprintf(html_buffer, sizeof(html_buffer), index_html, userData.ssid, userData.mbusinoName,
#ifdef MBUSINO_SENSORS
    userData.extension,
#endif
    userData.haAutodisc, userData.telegramDebug,
#ifdef MBUSINO_ETH
    ethModeSetting,
#else
    userData.sensorInterval/1000,
#endif
    userData.mbusInterval/1000, userData.broker, userData.mqttPort, userData.mqttUser
#if MBUSINO_SLAVES > 1
    , userData.mbusSlaves, userData.mbusAddress1, userData.mbusAddress2, userData.mbusAddress3, userData.mbusAddress4, userData.mbusAddress5
#endif
  );
}

void setupUpdateServer(){
  // Simple Firmware Update Form
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html",  (uint8_t *)update_html, update_htmlLength);
  });
  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
    waitForRestart = !Update.hasError();
    if(Update.hasError()==true){
      timerReboot = millis();
    }
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", waitForRestart?"success, restart now":"FAIL");
    response->addHeader("Connection", "close");
    request->send(response);
  },[](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
    if(!index){
#if defined(ESP8266)
      Update.runAsync(true);
#endif
      if(!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)){
        Update.printError(Serial);
      }
    }
    if(!Update.hasError()){
      if(Update.write(data, len) != len){
        Update.printError(Serial);
      }
    }
    if(final){
      if(Update.end(true)){
        LOG("Update Success: %uB\n", index+len);
      } else {
        Update.printError(Serial);
      }
    }
  });
}

void publishSettings(){
  client.publish(String(String(userData.mbusinoName) + "/settings/ssid").c_str(), userData.ssid);
  client.publish(String(String(userData.mbusinoName) + "/settings/broker").c_str(), userData.broker);
  client.publish(String(String(userData.mbusinoName) + "/settings/port").c_str(), String(userData.mqttPort).c_str());
  client.publish(String(String(userData.mbusinoName) + "/settings/user").c_str(), userData.mqttUser);
  client.publish(String(String(userData.mbusinoName) + "/settings/name").c_str(), userData.mbusinoName);
  client.publish(String(String(userData.mbusinoName) + "/settings/mbusInterval").c_str(), String(userData.mbusInterval).c_str());
#if MBUSINO_SLAVES > 1
  client.publish(String(String(userData.mbusinoName) + "/settings/slaves").c_str(), String(userData.mbusSlaves).c_str());
  for(uint8_t i = 0; i < MBUSINO_SLAVES; i++){
    client.publish(String(String(userData.mbusinoName) + "/settings/address" + String(i+1)).c_str(), String(mbusAddress[i]).c_str());
  }
#endif
  client.publish(String(String(userData.mbusinoName) + "/settings/MQTTreconnections").c_str(), String(conCounter-1).c_str());
  hwPublishSettings();
  client.publish(String(String(userData.mbusinoName) + "/settings/version").c_str(), MBUSINO_VERSION);
  client.publish(String(String(userData.mbusinoName) + "/settings/adcounter").c_str(), String(adMbusMessageCounter).c_str());
  client.publish(String(String(userData.mbusinoName) + "/settings/freeHeap").c_str(), String(ESP.getFreeHeap()).c_str());
  client.publish(String(String(userData.mbusinoName) + "/settings/minFreeHeap").c_str(), String(minFreeHeap).c_str());
}

void setup() {
  hwSetupEarly();
  mbus.begin();
  minFreeHeap = ESP.getFreeHeap();

  EEPROM.begin(512);
  loadSettings();
  hwLoadEeprom();
  EEPROM.commit();
  EEPROM.end();

  buildSetupPage();

  hwNetworkSetup();
  mbusLogBegin();
  client.setServer(userData.broker, userData.mqttPort);
  client.setCallback(callback);

  setupServer();
  if(apMode==true){
    dnsServer.start(53, "*", WiFi.softAPIP());
  }
  setupUpdateServer();
  // only requests to the setup AP, ON_AP_FILTER would also match Ethernet (WiFi.localIP() is 0.0.0.0 there)
  server.addHandler(new CaptiveRequestHandler()).setFilter([](AsyncWebServerRequest *request){
    return apMode == true && request->client()->localIP() == WiFi.softAPIP();
  });

  ArduinoOTA.setPassword((const char *)"mbusino");
#ifdef MBUSINO_ETH
  ArduinoOTA.setMdnsEnabled(false); // the mDNS component crashes (IGMP join in lwIP) when a netif appears or is torn down while it is running, e.g. Ethernet coming up while the setup AP is shutting down; MBusino does not use mDNS
#endif
  server.onNotFound(onRequest);
#if defined(ESP8266)
  ArduinoOTA.begin(&server);
#else
  ArduinoOTA.begin();
#endif
  server.begin();

  client.setBufferSize(6000);
  mbusSetup();
  hwSetupLate();
}

void loop() {
  heapCalc();
  ArduinoOTA.handle();
  hwLoop();

  if(credentialsReceived == true && waitForRestart == false){
    LOG("credentials received, save and restart soon\n");
    EEPROM.begin(512);
    EEPROM.put(100, userData);
    credentialsSaved = EEPROM_CREDENTIALS_NEW;
    EEPROM.put(eeAddrCredentialsSaved, credentialsSaved);
    hwSaveEeprom();
    EEPROM.commit();
    EEPROM.end();
    timerReboot = millis();
    waitForRestart = true;
  }

  if(waitForRestart==true && (millis() - timerReboot) > 1500){ // time to deliver the response page
    LOG("restart\n");
    ESP.restart();
  }

  if(hwMqttAllowed() == false){
    return;
  }
  if (!client.connected()) {
    if((millis() - timerReconnect) > 5000){
      hwReconnect();
      timerReconnect = millis();
    }
    return;
  }

  // the main code runs only if MQTT is connected
  client.loop();
#if MBUSINO_SLAVES > 1
  mbusSetAddressLoop();
#endif

  if((millis()-timerDebug) > 10000){
    timerDebug = millis();
    publishSettings();
  }

  hwConnectedLoop();
  mbusLoop();
  heapCalc();
}

void heapCalc(){
  if(minFreeHeap > ESP.getFreeHeap()){
    minFreeHeap = ESP.getFreeHeap();
  }
}
