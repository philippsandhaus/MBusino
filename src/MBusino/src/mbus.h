// M-Bus: polls the slaves one after the other and publishes the decoded records via MQTT.
// MBUSINO_SLAVES 1 polls the broadcast address 254, MBUSINO_SLAVES 5 the up to 5 configured addresses.
// The topics are kept as before: <name>/MBus/... for one slave, <name>/MBus/SlaveAddress<address>/... for more.

/*
mbusLoopStatus
0 = preparation of addresses ect. for the next switch loop
1 = if needed, M-Bus normalisation / initialisation (FCB set to 0), else direct to case 3
2 = only after a normalisation, wait for the answer (ACK)
3 = request the records from the slave
4 = wait for the mbus response
5 = get response from the rx buffer and decode the telegram
6 = Send decoded M-Bus records via MQTT
*/

#define MBUS_SLOTS (MBUSINO_SLAVES + 1) // the last slot is used for polling an address that is not configured

uint8_t mbusAddress[MBUSINO_SLAVES] = {MBUS_ADDRESS};
bool fcb[MBUS_SLOTS] = {false};             // M-Bus Frame Count Bit
bool initializeSlave[MBUS_SLOTS] = {false}; // m-bus normalizing is needed, set for all slaves in mbusSetup()
uint8_t recordCounter[MBUS_SLOTS] = {0};    // count the received records for multible telegrams

uint8_t mbusLoopStatus = 0;
uint8_t fields = 0;
char jsonstring[4096] = { 0 };
uint8_t responseAddress = 0; // address in the response telegram
bool engelmann = false;
bool polling = false;    // poll request via MQTT
bool mtPolling = false;  // another telegram with more records is available
bool shc = true;         // Switch Has Changed, for the debug output

uint8_t pollingAddress = MBUS_ADDRESS;
uint8_t currentAddress = 0;
uint8_t addressCounter = 0;
uint8_t currentAddressCounter = 0;

unsigned long timerMbus = 0;
unsigned long timerInitialize = 0;
unsigned long timerSerialAvailable = 0;
unsigned long timerMbusDecoded = 0;
unsigned long timerMbusReq = 0;

#if MBUSINO_SLAVES > 1
uint8_t newAddress = 0;
bool newAddressReceived = false;
bool waitToSetAddress = false;
unsigned long timerSetAddress = 0;
#endif

uint8_t mbusSlaveCount(){
#if MBUSINO_SLAVES > 1
  if(userData.mbusSlaves < 1) return 1;
  if(userData.mbusSlaves > MBUSINO_SLAVES) return MBUSINO_SLAVES;
  return userData.mbusSlaves;
#else
  return 1;
#endif
}

// topic prefix for the values of one slave
String mbusTopic(uint8_t address){
#if MBUSINO_SLAVES > 1
  return String(userData.mbusinoName) + "/MBus/SlaveAddress" + String(address) + "/";
#else
  return String(userData.mbusinoName) + "/MBus/";
#endif
}

// debug output, only if "publish M-Bus telegram" is switched on in the web interface
void mbusDebug(const char* topic, String value){
  if(userData.telegramDebug == true){
    client.publish((String(userData.mbusinoName) + "/MBus/debug/" + topic).c_str(), value.c_str());
  }
}

void mbusSetup(){
#if MBUSINO_SLAVES > 1
  mbusAddress[0] = userData.mbusAddress1;
  mbusAddress[1] = userData.mbusAddress2;
  mbusAddress[2] = userData.mbusAddress3;
  mbusAddress[3] = userData.mbusAddress4;
  mbusAddress[4] = userData.mbusAddress5;
#endif
  for(uint8_t i = 0; i < MBUS_SLOTS; i++){
    initializeSlave[i] = true;
  }
}

// publish the whole telegram in HEX, only for debugging
void mbusPublishTelegram(const byte* mbus_data){
  char telegram[MBUS_DATA_SIZE * 3 + 1] = {0};
  uint16_t len = mbus_data[1] + 6;
  if(len > MBUS_DATA_SIZE){
    len = MBUS_DATA_SIZE;
  }
  for(uint16_t i = 0; i < len; i++){
    sprintf(&telegram[i * 3], "%02X ", mbus_data[i]);
  }
#if MBUSINO_SLAVES > 1
  client.publish((String(userData.mbusinoName) + "/MBus/debug/telegram" + String(currentAddress)).c_str(), telegram);
#else
  client.publish((String(userData.mbusinoName) + "/debug/telegram").c_str(), telegram);
#endif
}

#if MBUSINO_SLAVES > 1
// set the address of a single connected slave, requested via the web interface (/setaddress)
void mbusSetAddressLoop(){
  if(newAddressReceived == true){
    newAddressReceived = false;
    waitToSetAddress = true;
    timerSetAddress = millis();
    mbus.normalize(254);
    client.publish(String(String(userData.mbusinoName) + "/setAddress/1").c_str(), "done");
  }
  if(waitToSetAddress == true && (millis() - 500) > timerSetAddress){
    waitToSetAddress = false;
    mbus.set_address(254,newAddress);
    client.publish(String(String(userData.mbusinoName) + "/setAddress/2").c_str(), String(newAddress).c_str());
  }
}
#endif

void mbusLoop(){
  if(shc){ // print the current mbusLoopStatus
    mbusDebug("mbus_Loop", String(mbusLoopStatus));
    shc = false;
  }

  switch(mbusLoopStatus){
    case 0:
      if((millis() - timerMbus) > userData.mbusInterval || polling == true || mtPolling == true){
        if(polling == false && mtPolling == false){ // if polling is true or a following telegram of a multi telegram, dont touch the timer
          timerMbus = millis();
        }
        if(mtPolling == true){ // another telegram with more records of the current slave
          mtPolling = false;
        }else if(polling == true){
          polling = false;
#if MBUSINO_SLAVES > 1
          currentAddress = pollingAddress;
#else
          currentAddress = mbusAddress[0];
#endif
          currentAddressCounter = MBUSINO_SLAVES; // not configured address
          for(uint8_t i = 0; i < mbusSlaveCount(); i++){
            if(mbusAddress[i] == currentAddress){
              currentAddressCounter = i;
            }
          }
          recordCounter[currentAddressCounter] = 0;
        }else{ // preperation of address and counter for the next mbus loop
          if(addressCounter >= mbusSlaveCount()){
            addressCounter = 0;
          }
          currentAddress = mbusAddress[addressCounter];
          currentAddressCounter = addressCounter;
          addressCounter++;
          recordCounter[currentAddressCounter] = 0;
        }
        mbusLoopStatus = 1;
        shc = true;
      }
      break;

    case 1:
      if(initializeSlave[currentAddressCounter] == true){
        initializeSlave[currentAddressCounter] = false;
        recordCounter[currentAddressCounter] = 0;
        mbus.clearRXbuffer();
        mbus.normalize(currentAddress);
        fcb[currentAddressCounter] = false;
        timerInitialize = millis();
        mbusLoopStatus = 2;
        mbusDebug("nomalize/address", String(currentAddress));
      }else{
        mbusLoopStatus = 3;
      }
      shc = true;
      break;

    case 2: // wait for the answer (ACK) of the M-Bus normalisation / initialisation
      if((millis() - timerInitialize) > 2000){
        if(mbus.available()){
          byte rxbuffer[256] = {0};
          uint8_t rxbuffercontent = mbus.read_rxbuffer(rxbuffer, sizeof(rxbuffer));
          if(rxbuffercontent == 1 && rxbuffer[0] == 0xE5) {
            mbusDebug("nomalize/ack", String(rxbuffer[0]));
          }else{
            mbusDebug("nomalize/no_ack", String(currentAddress));
          }
        }else{
          mbusDebug("nomalize/no_answer", String(currentAddress));
        }
        mbusLoopStatus = 3;
        shc = true;
      }
      break;

    case 3: // request the records from the slave
      mbus.clearRXbuffer();
      mbus.request_data(currentAddress, fcb[currentAddressCounter]);
      mbusDebug("request/currentAddress", String(currentAddress));
      mbusDebug("request/fcb", String(fcb[currentAddressCounter]));
      mbusLoopStatus = 4;
      shc = true;
      timerMbusReq = millis();
      break;

    case 4: // wait for the mbus response
      if(mbus.available()){
        mbusLoopStatus = 5;
        shc = true;
        timerSerialAvailable = millis();
      }
      else if(millis() - timerMbusReq > 2000){ // failure, no data received
        initializeSlave[currentAddressCounter] = true;
        recordCounter[currentAddressCounter] = 0;
        fcb[currentAddressCounter] = false;
        client.publish((mbusTopic(currentAddress) + "MBUSerror").c_str(), "no_Data_received");
        mbusLogAdd(MBUS_LOG_NO_DATA, currentAddress, NULL);
        mbusLoopStatus = 0;
        shc = true;
        mtPolling = false;
      }
      break;

    case 5: // get response from the rx buffer and decode the telegram
      if((millis() - timerSerialAvailable) > 1500){
        mbusLoopStatus = 6;
        shc = true;
        byte mbus_data[MBUS_DATA_SIZE] = { 0 };
        bool mbus_good_frame = mbus.get_response(mbus_data, sizeof(mbus_data));
        mbusLogAdd(mbus_good_frame ? MBUS_LOG_OK : MBUS_LOG_BAD_FRAME, currentAddress, mbus_data);

        if(userData.telegramDebug == true){
          mbusPublishTelegram(mbus_data);
        }

        if(mbus_good_frame){
          fcb[currentAddressCounter] = !fcb[currentAddressCounter]; // toggle the FCB (Frame Count Bit) to signalize good response in the next request

          int packet_size = mbus_data[1] + 6;
          JsonDocument doc;
          JsonObject headerObj = doc["header"].to<JsonObject>();
          payload.decodeHeader(mbus_data, packet_size, headerObj);
          JsonArray recordsArr = doc["records"].to<JsonArray>();
          fields = payload.decodeRecords(&mbus_data[Startadd], packet_size - Startadd - 2, recordsArr);
          responseAddress = mbus_data[5];

          serializeJson(doc, jsonstring); // store the json in a global array
          mbusLogSetJson(responseAddress, jsonstring);
          client.publish((mbusTopic(responseAddress) + "error").c_str(), String(payload.getError()).c_str());
          client.publish((mbusTopic(responseAddress) + "jsonstring").c_str(), jsonstring);
          client.publish((mbusTopic(responseAddress) + (MBUSINO_SLAVES > 1 ? "fcb" : "FCB")).c_str(), String(fcb[currentAddressCounter]).c_str());
          heapCalc();
          engelmann = (mbus_data[12]==0x14 && mbus_data[11]==0xC5);
          timerMbusDecoded = millis();
        }else{
          initializeSlave[currentAddressCounter] = true;
          recordCounter[currentAddressCounter] = 0;
          fcb[currentAddressCounter] = false;
          jsonstring[0] = 0;
          client.publish((mbusTopic(currentAddress) + "MBUSerror").c_str(), "no_good_telegram");
          mbusLoopStatus = 0;
          mtPolling = false;
        }
      }
      break;

    case 6: // Send decoded M-Bus records via MQTT
      if(millis() - timerMbusDecoded > 100){
        mbusLoopStatus = 0;
        shc = true;
        JsonDocument doc;
        deserializeJson(doc, jsonstring); // load the json from the global array
        JsonObject headerObj = doc["header"];
        JsonArray recordsArr = doc["records"];
        jsonstring[0] = 0;
        String topic = mbusTopic(responseAddress);

        // Publish M-Bus header via MQTT
        if (!headerObj.isNull()) {
          for(uint8_t i = 0; i < HEADER_PUBLISH_COUNT; i++){
            char valBuf[20] = {0};
            const char* val = nullptr;
            switch(headerPublishFields[i].type){
              case HT_STR: val = headerObj[headerPublishFields[i].jsonKey].as<const char*>(); break;
              case HT_INT: snprintf(valBuf, sizeof(valBuf), "%d", headerObj[headerPublishFields[i].jsonKey].as<int>()); val = valBuf; break;
              case HT_HEX: snprintf(valBuf, sizeof(valBuf), "%x", headerObj[headerPublishFields[i].jsonKey].as<int>()); val = valBuf; break;
              default: break;
            }
            if(val) client.publish((topic + "header/" + headerPublishFields[i].jsonKey).c_str(), val);
          }
          JsonObject sd = headerObj["status_details"];
          for(uint8_t i = 0; i < HEADER_STATUS_COUNT; i++){
            client.publish((topic + "header/" + headerStatusKeys[i]).c_str(), sd[headerStatusKeys[i]].as<bool>() ? "on" : "off");
          }
        }

        // Header autodiscovery
        if(userData.haAutodisc == true && adMbusMessageCounter == 3){
          haHandoverHeader(responseAddress);
        }

        uint8_t recordOffset = recordCounter[currentAddressCounter];
        for (uint8_t i=0; i<fields; i++) {
          uint8_t code = recordsArr[i]["code"].as<int>();
          const char* name = recordsArr[i]["name"];
          const char* units = recordsArr[i]["units"];
          double value = recordsArr[i]["value_scaled"].as<double>();
          const char* valueString = recordsArr[i]["value_string"];
          bool telegramFollow = recordsArr[i]["telegramFollow"].as<int>();
          String recordTopic = topic + String(recordOffset+i+1) + "_";

          if(userData.haAutodisc == true && adMbusMessageCounter == 3){  // every 256th readout is a HA autoconfig message
            strcpy(adVariables.haName,name);
            strcpy(adVariables.haUnits, units != NULL ? units : "");
            strcpy(adVariables.stateClass,payload.getStateClass(code));
            strcpy(adVariables.deviceClass,payload.getDeviceClass(code, (uint8_t)headerObj["medium_code"].as<int>()));
            haHandoverMbus(recordOffset+i+1, engelmann, responseAddress);
          }else{
            //two messages per value, values comes as number or as ASCII string or both
            if(valueString != NULL && strlen(valueString) > 0){
              client.publish((recordTopic + "vs_" + String(name)).c_str(), valueString); // send the value if a ascii value is aviable (variable length)
            }
            client.publish((recordTopic + String(name)).c_str(), String(value,3).c_str()); // send the value if a real value is aviable (standard)
            if(units != NULL && strlen(units) > 0){
              client.publish((recordTopic + String(name) + "_unit").c_str(), units);
            }

            if (i == 3 && engelmann == true){  // Sensostar Bugfix --> comment it out if you use not a Sensostar
              float flow = recordsArr[5]["value_scaled"].as<float>();
              float delta = recordsArr[9]["value_scaled"].as<float>();
              float calc_power = delta * flow * 1163;
              client.publish((topic + "4_power_calc").c_str(), String(calc_power).c_str());
            }
          }

          if(fields == i+1){ // ...last Field
            if(telegramFollow == 1){ // another telegram with more records is available
              client.publish((topic + String(recordOffset+i+1)).c_str(),"--> More records follow in next telegram");
              recordCounter[currentAddressCounter] = recordOffset + fields; // add the current number of recieved records to the previusly sended numbers of records
              mtPolling = true;
            }else{
              recordCounter[currentAddressCounter] = 0;
              mtPolling = false;
              if(addressCounter == 1){ // one complete round over all slaves
                adMbusMessageCounter++;
              }
            }
          }
        }
        heapCalc();
      }
      break;
  }
}
