String mbusTopic(uint8_t address); // mbus.h
uint8_t mbusSlaveCount();       // mbus.h

struct autodiscover {
  char bufferValue[1024] = {0};
  char bufferTopic[100] = {0};
  char haName[30] = {0};
  char haUnits[30] = {0};
  char stateClass[30] = {0};
  char deviceClass[30] = {0};
  char deviceClassString[50] = {0};
  char stateClassString[50] = {0};
  char unitString[50] = {0};
} adVariables; // home assistand auto discover

#ifdef MBUSINO_SENSORS
const char bmeValue[4][12] = {"temperature","pressure","altitude","humidity"};
const char bmeDeviceClass[4][12] = {"temperature","pressure","distance","humidity"};
const char bmeUnits[4][5] = {"°C","mbar","m","%"};
#endif

// M-Bus entities: unique id, state topic and name are kept as before,
// "<n>_<name>" with one slave, "<address>_<n>_<name>" (name "Addr<address>_<n>_<name>") with more
const char adValueMbus[] PROGMEM = R"rawliteral({"unique_id":"%s_%s","default_entity_id":"sensor.%s_%s","state_topic":"%s","name":"%s","value_template":"{{value_json}}",%s%s"device":{"ids": ["%s"],"name":"%s","manufacturer": "MBusino","mdl":"V%s"},%s"expire_after":%u,"availability_topic":"%s/status","payload_available":"online","payload_not_available":"offline","availability_mode":"all"})rawliteral";
const char adTopicMbus[] PROGMEM = R"rawliteral(homeassistant/sensor/%s/%s/config)rawliteral";

String haMbusId(uint8_t address, String suffix){
#if MBUSINO_SLAVES > 1
  return String(address) + "_" + suffix;
#else
  return suffix;
#endif
}

String haMbusName(uint8_t address, String suffix){
#if MBUSINO_SLAVES > 1
  return "Addr" + String(address) + "_" + suffix;
#else
  return suffix;
#endif
}

void haPublishMbusEntity(uint8_t address, String suffix){
  String id = haMbusId(address, suffix);
  sprintf(adVariables.bufferValue, adValueMbus, userData.mbusinoName, id.c_str(), userData.mbusinoName, id.c_str(), (mbusTopic(address) + suffix).c_str(), haMbusName(address, suffix).c_str(),
          adVariables.unitString, adVariables.stateClassString, userData.mbusinoName, userData.mbusinoName, MBUSINO_VERSION, adVariables.deviceClassString,
          userData.mbusInterval / 1000 * 3 * mbusSlaveCount(), userData.mbusinoName); // each slave is polled once per round over all slaves
  sprintf(adVariables.bufferTopic, adTopicMbus, userData.mbusinoName, id.c_str());
  client.publish(adVariables.bufferTopic, adVariables.bufferValue, true);
}

void haHandoverMbus(uint8_t haCounter, bool engelmann, uint8_t address){ // haCounter is the record number
  if(adVariables.deviceClass[0] != 0){
    strcpy(adVariables.deviceClassString,String("\"device_class\": \"" + String(adVariables.deviceClass) + "\",").c_str());
  }
  if(adVariables.stateClass[0] != 0){
    strcpy(adVariables.stateClassString,String("\"state_class\": \"" + String(adVariables.stateClass) + "\"," ).c_str());
  }
  if(adVariables.haUnits[0] != 0){
    strcpy(adVariables.unitString,String("\"unit_of_meas\": \"" + String(adVariables.haUnits) + "\"," ).c_str());
  }
  haPublishMbusEntity(address, String(haCounter) + "_" + String(adVariables.haName));

  if (haCounter == 4 && engelmann == true){  // Sensostar Bugfix --> comment it out if you use not a Sensostar
    haPublishMbusEntity(address, String(haCounter) + "_power_calc");
  }

  adVariables.bufferTopic[0] = 0;
  adVariables.bufferValue[0] = 0;
  adVariables.deviceClass[0] = 0;
  adVariables.deviceClassString[0] = 0;
  adVariables.stateClass[0] = 0;
  adVariables.stateClassString[0] = 0;
  adVariables.haUnits[0] = 0;
  adVariables.unitString[0] = 0;
}

#ifdef MBUSINO_SENSORS
const char adValueSensor[] PROGMEM = R"rawliteral({"unique_id":"%s_Sensor%u","default_entity_id":"sensor.%s_Sensor%u","state_topic":"%s/OneWire/S%u","name":"Sensor%u","value_template":"{{value_json}}","unit_of_meas":"°C","state_class":"measurement","device":{"ids": ["%s"],"name":"%s","manufacturer": "MBusino","mdl":"V%s"},"device_class":"temperature","expire_after":%u,"availability_topic":"%s/status","payload_available":"online","payload_not_available":"offline","availability_mode":"all"})rawliteral";
const char adTopicSensor[] PROGMEM = R"rawliteral(homeassistant/sensor/%s/Sensor%u/config)rawliteral";

const char adValueBME[] PROGMEM = R"rawliteral({"unique_id":"%s__BME_%s","default_entity_id":"sensor.%s_BME_%s","state_topic":"%s/bme/%s","name":"%s","value_template":"{{value_json}}","unit_of_meas":"%s","state_class":"measurement","device":{"ids": ["%s"],"name":"%s","manufacturer": "MBusino","mdl":"V%s"},"device_class":"%s","expire_after":%u,"availability_topic":"%s/status","payload_available":"online","payload_not_available":"offline","availability_mode":"all"})rawliteral";
const char adTopicBME[] PROGMEM = R"rawliteral(homeassistant/sensor/%s/%s/config)rawliteral";

void haHandoverOw(uint8_t haCounter){
  sprintf(adVariables.bufferValue,adValueSensor,userData.mbusinoName,haCounter,userData.mbusinoName,haCounter,userData.mbusinoName,haCounter,haCounter,userData.mbusinoName,userData.mbusinoName,MBUSINO_VERSION,userData.sensorInterval / 1000 * 3,userData.mbusinoName);
  sprintf(adVariables.bufferTopic,adTopicSensor,userData.mbusinoName,haCounter);
  client.publish(adVariables.bufferTopic, adVariables.bufferValue, true); 
  adVariables.bufferTopic[0] = 0;
  adVariables.bufferValue[0] = 0;
}

void haHandoverBME(){
  for(uint8_t i=0; i<4;i++){  
    sprintf(adVariables.bufferValue,adValueBME,userData.mbusinoName,bmeValue[i],userData.mbusinoName,bmeValue[i],userData.mbusinoName,bmeValue[i],bmeValue[i],bmeUnits[i],userData.mbusinoName,userData.mbusinoName,MBUSINO_VERSION,bmeDeviceClass[i],userData.sensorInterval / 1000 * 3,userData.mbusinoName);
    sprintf(adVariables.bufferTopic,adTopicBME,userData.mbusinoName,bmeValue[i]);
    client.publish(adVariables.bufferTopic, adVariables.bufferValue, true); 
    adVariables.bufferTopic[0] = 0;
    adVariables.bufferValue[0] = 0;
  }
}
#endif

// --- Header autodiscovery ---
struct headerAdField {
  const char* jsonKey;
};

static const headerAdField headerAdFields[] = {
  {"address"},
  {"id"},
  {"manufacturer"},
  {"medium"},
  {"version"},
  {"status"},
  {"access_counter"},
  {"battery_low"},
  {"temporary_error"},
  {"permanent_error"},
};

#define HEADER_AD_FIELDS_COUNT (sizeof(headerAdFields) / sizeof(headerAdFields[0]))

const char adValueHeader[] PROGMEM = R"rawliteral({"unique_id":"%s_header_%s","default_entity_id":"sensor.%s_header_%s","state_topic":"%s","name":"%s","value_template":"{{value}}","device":{"ids": ["%s"],"name":"%s","manufacturer": "MBusino","mdl":"V%s"},"availability_topic":"%s/status","payload_available":"online","payload_not_available":"offline","availability_mode":"all"})rawliteral";
const char adTopicHeader[] PROGMEM = R"rawliteral(homeassistant/sensor/%s/header_%s/config)rawliteral";

void haHandoverHeader(uint8_t address){
  for(uint8_t i = 0; i < HEADER_AD_FIELDS_COUNT; i++){
    String id = haMbusId(address, headerAdFields[i].jsonKey);
    sprintf(adVariables.bufferValue, adValueHeader,
      userData.mbusinoName, id.c_str(),
      userData.mbusinoName, id.c_str(),
      (mbusTopic(address) + "header/" + headerAdFields[i].jsonKey).c_str(),
      haMbusName(address, headerAdFields[i].jsonKey).c_str(),
      userData.mbusinoName, userData.mbusinoName, MBUSINO_VERSION,
      userData.mbusinoName);
    sprintf(adVariables.bufferTopic, adTopicHeader, userData.mbusinoName, id.c_str());
    client.publish(adVariables.bufferTopic, adVariables.bufferValue, true);
    adVariables.bufferTopic[0] = 0;
    adVariables.bufferValue[0] = 0;
  }
}

// --- Header MQTT publish ---
enum HeaderFieldType { HT_STR, HT_INT, HT_HEX, HT_BOOL_NESTED };

struct headerPublishField {
  const char* jsonKey;
  HeaderFieldType type;
};

static const headerPublishField headerPublishFields[] = {
  {"address",        HT_INT},
  {"id",             HT_STR},
  {"manufacturer",   HT_STR},
  {"medium",         HT_STR},
  {"version",        HT_INT},
  {"status",         HT_HEX},
  {"access_counter", HT_INT},
};

#define HEADER_PUBLISH_COUNT (sizeof(headerPublishFields) / sizeof(headerPublishFields[0]))

static const char* headerStatusKeys[] = {"battery_low", "temporary_error", "permanent_error"};
#define HEADER_STATUS_COUNT (sizeof(headerStatusKeys) / sizeof(headerStatusKeys[0]))
