void reconnect() {
  LOG("MQTT try to connect server\n");
  char lwBuffer[40] = {0};
  snprintf(lwBuffer, sizeof(lwBuffer), "%s/status", userData.mbusinoName);
  if (client.connect(userData.mbusinoName,userData.mqttUser,userData.mqttPswrd,lwBuffer,1,true,"offline")) {
    // Once connected, publish an announcement...
    LOG("MQTT connected to server\n");
    ledMqtt(true);
    conCounter++;
    if(conCounter == 1){
      client.publish(String(String(userData.mbusinoName) + "/start").c_str(), "Bin hochgefahren, WLAN und MQTT steht");
    }
    else{
      client.publish(String(String(userData.mbusinoName) + "/reconnect").c_str(), "Online again!");
      adMbusMessageCounter = 2;
      hwMqttReconnected();
    }
    // publish online status for HA availability (retained)
    client.publish(lwBuffer, "online", true);
    // ... and resubscribe
    hwMqttSubscribe();
    client.subscribe(String(String(userData.mbusinoName) + "/mbusPolling").c_str());
  }
  else{
    LOG("MQTT unable to connect server\n");
    ledMqtt(false);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  hwMqttCallback(topic, payload);
  if (strcmp(topic,String(String(userData.mbusinoName) + "/mbusPolling").c_str())==0){
    char address[4] = {0}; // the payload is not null terminated
    memcpy(address, payload, length < sizeof(address) ? length : sizeof(address) - 1);
    pollingAddress = atoi(address); // the address to poll, only used with more than one slave
    polling = true;
  }
}
