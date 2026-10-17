void reconnect() {
  // Loop until we're reconnected
#ifdef MBUSINO_ETH
  Serial.println("MQTT try to connect server");
#endif
  char lwBuffer[40] = {0};
  snprintf(lwBuffer, sizeof(lwBuffer), "%s/status", userData.mbusinoName);
  if (client.connect(userData.mbusinoName,userData.mqttUser,userData.mqttPswrd,lwBuffer,1,true,"offline")) {
    // Once connected, publish an announcement...
#ifdef MBUSINO_ETH
    Serial.println("MQTT connected to server");
    digitalWrite(LED_BUILTIN,LOW);
#else
    #if defined(ESP32)
    Serial.println("MQTT connected to server");
    #endif
#endif
    conCounter++;
#if defined(MBUSINO_SENSORS) && MBUSINO_SLAVES == 1
    pulseInterval = 50;
#endif
    if(conCounter == 1){
      client.publish(String(String(userData.mbusinoName) + "/start").c_str(), "Bin hochgefahren, WLAN und MQTT steht");
    }
    else{
      client.publish(String(String(userData.mbusinoName) + "/reconnect").c_str(), "Online again!");
      adMbusMessageCounter = 2;
#ifdef MBUSINO_SENSORS
      adSensorMessageCounter = 2;
#endif
    }
    // publish online status for HA availability (retained)
    client.publish(lwBuffer, "online", true);
    // ... and resubscribe
#ifdef MBUSINO_SENSORS
    client.subscribe(String(String(userData.mbusinoName) + "/calibrateAverage").c_str());
    client.subscribe(String(String(userData.mbusinoName) + "/calibrateSensor").c_str());
    client.subscribe(String(String(userData.mbusinoName) + "/calibrateValue").c_str());
    client.subscribe(String(String(userData.mbusinoName) + "/calibrateBME").c_str());
    client.subscribe(String(String(userData.mbusinoName) + "/calibrateSet0").c_str());
#endif
    client.subscribe(String(String(userData.mbusinoName) + "/mbusPolling").c_str());
  }
  else{
#ifdef MBUSINO_ETH
    Serial.println("MQTT unable to connect server");
    digitalWrite(LED_BUILTIN,HIGH);
#else
    #if defined(ESP32)
    Serial.println("MQTT unable to connect server");
    #endif
    pulseInterval = 400;
#endif
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
#ifdef MBUSINO_SENSORS
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
#endif
  if (strcmp(topic,String(String(userData.mbusinoName) + "/mbusPolling").c_str())==0){
#if defined(MBUSINO_SENSORS) && MBUSINO_SLAVES > 1
    pollingAddress = atoi((char*)payload);
#endif
    polling = true;
  }
}
