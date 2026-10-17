// MBusino shield: ESP8266 D1 mini or ESP32 S2 mini with Wi-Fi, OneWire (DS18B20) and BME280.
// The LED pulses, the pulse interval shows the network status.

#define EEPROM_CALIBRATED_FLAG 500

#if defined(ESP8266)
#define ONE_WIRE_BUS1 2   //D4
#define ONE_WIRE_BUS2 13  //D7
#define ONE_WIRE_BUS3 12  //D6
#define ONE_WIRE_BUS4 14  //D5
#define ONE_WIRE_BUS5 0   //D3
#define ONE_WIRE_BUS6 5   //D1
#define ONE_WIRE_BUS7 4   //D2
#elif defined(ESP32)
#define ONE_WIRE_BUS1 16 //2   //D4
#define ONE_WIRE_BUS2 11  //13  //D7
#define ONE_WIRE_BUS3 9   //12  //D6
#define ONE_WIRE_BUS4 7   //14  //D5
#define ONE_WIRE_BUS5 18  //0   //D3
#define ONE_WIRE_BUS6 35  //5   //D1
#define ONE_WIRE_BUS7 33  //4   //D2
#endif

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;  // I2C

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire1(ONE_WIRE_BUS1);
OneWire oneWire2(ONE_WIRE_BUS2);
OneWire oneWire3(ONE_WIRE_BUS3);
OneWire oneWire4(ONE_WIRE_BUS4);
OneWire oneWire5(ONE_WIRE_BUS5);
OneWire oneWire6(ONE_WIRE_BUS6);
OneWire oneWire7(ONE_WIRE_BUS7);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensor1(&oneWire1);
DallasTemperature sensor2(&oneWire2);
DallasTemperature sensor3(&oneWire3);
DallasTemperature sensor4(&oneWire4);
DallasTemperature sensor5(&oneWire5);
DallasTemperature sensor6(&oneWire6);
DallasTemperature sensor7(&oneWire7);

float OW[7] = {0};         // variables for DS18B20 Onewire sensors
float temperatur = 0;      // Variablen für den BLE280 am I2C
float druck = 0;
float hoehe = 0;
float feuchte = 0;
bool bmeStatus;
uint8_t sensorStatus = 0;

uint8_t eeAddrCalibrated = 0;
uint8_t eeAddrOffset[7] = {4,8,12,16,20,24,28};  //EEPROM address to start reading from
uint16_t calibrated = 123;  // shows if EEPROM used befor for offsets
float offset[7] = {0};
float OWwO[7] = {0};  // Variables for DS18B20 Onewire Sensores with Offset (One Wire1 with Offset)
bool OWnotconnected[7] = {false};
uint8_t sensorToCalibrate = 0;
uint8_t adSensorMessageCounter = 0; // Counter for autodiscouver sensor message.

unsigned long timerMQTT = 15000;
unsigned long timerSensorRefresh1 = 0;
unsigned long timerPulse = 0;
uint16_t pulseInterval = 1000;
bool ledStatus = false;

#include "calibration.h"
#include "sensorRefresh.h"

void ledPulse(uint16_t interval){
  pulseInterval = interval;
}

void ledMqtt(bool connected){
  ledPulse(connected ? 50 : 400);
}

void hwSetupEarly(){
#if defined(ESP32) // on the ESP8266 Serial is the M-Bus UART
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(1500);
#endif
}

// read the OneWire offsets, EEPROM.begin() is already called
void hwLoadEeprom(){
  EEPROM.get(eeAddrCalibrated, calibrated);
  if(calibrated==EEPROM_CALIBRATED_FLAG){ // if EEPROM_CALIBRATED_FLAG the EEPROM is not used before and full of junk
    for(uint8_t i = 0; i <=userData.extension; i++){
      EEPROM.get(eeAddrOffset[i], offset[i]);
    }
  }
  else{
    for(uint8_t i = 0; i < 7; i++){
      EEPROM.put(eeAddrOffset[i], 0);  // copy offset 0 to EEPROM
    }
    calibrated = EEPROM_CALIBRATED_FLAG;
    EEPROM.put(eeAddrCalibrated, calibrated);  // copy the key to EEPROM that the EEPROM is written and not full of junk.
  }
}

void hwSaveEeprom(){}

void hwNetworkSetup(){
#if defined(ESP32)
  WiFi.onEvent(WiFiEvent);
#endif
  WiFi.hostname(userData.mbusinoName);
  WiFi.mode(WIFI_STA);
  WiFi.begin(userData.ssid, userData.password);

  byte tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    if (tries++ > 10) {
      WiFi.mode(WIFI_AP);
      WiFi.softAP("MBusino Setup Portal"); //, "secret");
      apMode = true;
      break;
    }
  }
}

void hwSetupLate(){
  // Start up the library
  if(userData.extension > 0){
    sensor1.begin();
    sensor2.begin();
    sensor3.begin();
    sensor4.begin();
    sensor5.begin();

    sensor1.setWaitForConversion(false);  // Request temperature conversion - non-blocking / async
    sensor2.setWaitForConversion(false);
    sensor3.setWaitForConversion(false);
    sensor4.setWaitForConversion(false);
    sensor5.setWaitForConversion(false);
  }
  if(userData.extension == 7){
    sensor6.begin();
    sensor7.begin();

    sensor6.setWaitForConversion(false);
    sensor7.setWaitForConversion(false);
  }

  if(userData.extension == 5){
    // Vorbereitungen für den BME280
    bmeStatus = bme.begin(0x76);
  }
}

// setup AP, LED pulse and Wi-Fi reconnect
void hwLoop(){
  if(apMode == true){
    dnsServer.processNextRequest();
  }
#if defined(ESP32)
  if(millis() - timerPulse >= pulseInterval){ // Blink of the internal LED to see MBusino is still alive and the network status
    timerPulse = millis();
    ledStatus = !ledStatus;
    digitalWrite(LED_BUILTIN, ledStatus ? HIGH : LOW);
  }

  if(wifiReconnect == true && (millis() - timerWifiReconnect > 2000)){
    Serial.println("try to reconnect");
    wifiReconnect = false;
    WiFi.reconnect();
  }
#endif

  if(apMode == true && millis() > 300000){
    ESP.restart();
  }
}

// MQTT may be used: only with a Wi-Fi connection, not while the setup AP is running
bool hwMqttAllowed(){
  return WiFi.status() == WL_CONNECTED && apMode == false;
}

void hwReconnect(){
  reconnect();
}

void hwPublishSettings(){
  client.publish(String(String(userData.mbusinoName) + "/settings/extension").c_str(), String(userData.extension).c_str());
  client.publish(String(String(userData.mbusinoName) + "/settings/sensorInterval").c_str(), String(userData.sensorInterval).c_str());
  client.publish(String(String(userData.mbusinoName) + "/settings/IP").c_str(), String(WiFi.localIP().toString()).c_str());
  client.publish(String(String(userData.mbusinoName) + "/settings/RSSI").c_str(), String(WiFi.RSSI()).c_str());
}

void hwMqttReconnected(){
  adSensorMessageCounter = 2;
}

void hwMqttSubscribe(){
  client.subscribe(String(String(userData.mbusinoName) + "/calibrateAverage").c_str());
  client.subscribe(String(String(userData.mbusinoName) + "/calibrateSensor").c_str());
  client.subscribe(String(String(userData.mbusinoName) + "/calibrateValue").c_str());
  client.subscribe(String(String(userData.mbusinoName) + "/calibrateBME").c_str());
  client.subscribe(String(String(userData.mbusinoName) + "/calibrateSet0").c_str());
}

void hwMqttCallback(char* topic, byte* payload){
  if(userData.extension > 0){
    if (strcmp(topic,String(String(userData.mbusinoName) + "/calibrateAverage").c_str())==0){
      calibrationAverage();
    }
    if (strcmp(topic,String(String(userData.mbusinoName) + "/calibrateSensor").c_str())==0){
      calibrationSensor(atoi((char*)payload)-1);
    }
    if (strcmp(topic,String(String(userData.mbusinoName) + "/calibrateValue").c_str())==0){
      calibrationValue(atof((char*)payload));
    }
    if(userData.extension == 5){
      if (strcmp(topic,String(String(userData.mbusinoName) + "/calibrateBME").c_str())==0){
        calibrationBME();
      }
    }
    if (strcmp(topic,String(String(userData.mbusinoName) + "/calibrateSet0").c_str())==0){
      calibrationSet0();
    }
  }
}

// runs while MQTT is connected: read the OneWire and BME280 sensors and publish them
void hwConnectedLoop(){
  if(userData.extension == 0){
    return;
  }
  switch(sensorStatus){
    case 0:
      if((millis() - timerMQTT) > userData.sensorInterval) { // springt in die Funktion zum anstoßen der aktuallisierung der Sensorwerte
        sensorRefresh1();
        timerMQTT = millis();
        sensorStatus = 1;
        timerSensorRefresh1 = millis();
      }
      break;
    case 1:
      if ((millis() - timerSensorRefresh1) > 800) {  // springt in die Funktion zum holen der neuen Sensorwerte
        sensorRefresh2();
        timerSensorRefresh1 = millis();
        sensorStatus = 2;
      }
      break;
    case 2:
      if((millis() - timerSensorRefresh1) > 200){
        adSensorMessageCounter++;
        for(uint8_t i = 0; i < userData.extension; i++){
          if(OW[i] != -127){
            client.publish(String(String(userData.mbusinoName) + "/OneWire/S" + String(i+1)).c_str(), String(OWwO[i]).c_str());
            client.publish(String(String(userData.mbusinoName) + "/OneWire/offset" + String(i+1)).c_str(), String(offset[i]).c_str());
            if(userData.haAutodisc == true && adSensorMessageCounter == 3){
              haHandoverOw(i+1);
            }
          }
        }

        if(userData.extension == 5){
          client.publish(String(String(userData.mbusinoName) + "/bme/temperature").c_str(), String(temperatur).c_str());
          client.publish(String(String(userData.mbusinoName) + "/bme/pressure").c_str(), String(druck).c_str());
          client.publish(String(String(userData.mbusinoName) + "/bme/altitude").c_str(), String(hoehe).c_str());
          client.publish(String(String(userData.mbusinoName) + "/bme/humidity").c_str(), String(feuchte).c_str());
          if(userData.haAutodisc == true && adSensorMessageCounter == 3){
            haHandoverBME();
          }
        }
        sensorStatus = 0;
      }
      break;
  }
}
