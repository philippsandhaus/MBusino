// MBusino Nano: ESP32 C3 SuperMini with W5500 Ethernet, M-Bus only.
// Ethernet is prioritized over Wi-Fi, exactly one transport is active: Ethernet > Wi-Fi > setup AP.
// The LED is on while MQTT is connected and blinks while the setup AP is running.

#define EEPROM_ETH_LINK_MODE 40 // last working Ethernet link mode, 1 = forced 10M full duplex, otherwise auto negotiation
#define EEPROM_ETH_MODE_SETTING 41 // Ethernet link mode chosen in the web interface, see ETH_MODE_...
#define ETH_DHCP_TIMEOUT 15000 // ms after link up without IP before the other link mode is tried
#define ETH_LINK_TIMEOUT 8000 // ms after ETH.begin() without link up before the other link mode is tried. Link up is decided at PHY level well below 1 s, so 8 s is already a generous margin; the previous 20 s only delayed the fallback for strict equipment (Fritzbox 7590, PoE extenders)
#define ETH_LOSS_GRACE 15000 // ms an already working Ethernet connection (IP) may stay away before a controlled restart; the boot then re-picks the transport (Ethernet -> Wi-Fi -> setup AP)

#define ETH_MODE_AUTO 0      // try auto negotiation and forced 10M full duplex, remember the working one
#define ETH_MODE_AUTONEG 1   // always auto negotiation
#define ETH_MODE_FORCED10M 2 // always forced 10M full duplex

// Pins for an ESP32 C3 Supermini standard SPI
#ifndef ETH_PHY_CS
#define ETH_PHY_TYPE     ETH_PHY_W5500
#define ETH_PHY_ADDR     0
#define ETH_PHY_CS       7
#define ETH_PHY_IRQ      2
#define ETH_PHY_RST      3
#define ETH_PHY_SPI_HOST SPI2_HOST
#define ETH_PHY_SPI_SCK  4
#define ETH_PHY_SPI_MISO 5
#define ETH_PHY_SPI_MOSI 6
#endif

/*
// Pins for an ESP32 C3 Supermini V01 SPI
#define ETH_PHY_ADDR     1
#define ETH_PHY_SPI_SCK  8
#define ETH_PHY_SPI_MISO 9
#define ETH_PHY_SPI_MOSI 10
*/

static bool eth_connected = false;
uint8_t usedMQTTconection = 0; // 1 = Ethernet, 2 = Wi-Fi
bool gotIP = false;
bool ethForced10M = false; // false = auto negotiation, true = forced 10M full duplex (needed by some strict equipment, e.g. Fritzbox 7590)
bool ethForced10MStored = false; // link mode saved in EEPROM
bool ethLinkSeen = false; // link was up at least once since the last ETH.begin()
bool ethModeConfirmed = false; // got an IP with the current link mode since start
uint8_t ethModeSetting = ETH_MODE_AUTO;
unsigned long timerETHbegin = 0;
unsigned long ethLostSince = 0; // millis() when an established Ethernet connection (IP) was lost, 0 = not lost; used for the restart grace period in loop()
unsigned long ethRestarting = 0; // millis() when loop() intentionally restarted ETH for a link mode change, 0 = not restarting; suppresses transport switching while ETH re-initialises
unsigned long timerNetworkChange = 0;
unsigned long timerETHmessage = 0;
unsigned long timerLedAPmode = 0;

void ledPulse(uint16_t interval){} // the Nano LED shows the MQTT connection, see ledMqtt()

void ledMqtt(bool connected){
  digitalWrite(LED_BUILTIN, connected ? LOW : HIGH);
}

// apply the Ethernet link mode, must be called before every ETH.begin()
void setEthLinkMode(){
  ETH.setAutoNegotiation(!ethForced10M);
  if(ethForced10M == true){
    ETH.setFullDuplex(true);
    ETH.setLinkSpeed(10);
  }
  ethLinkSeen = false;
  timerETHbegin = millis();
  Serial.printf("[ETH] %lu ms: link mode applied: %s\n", millis(), ethForced10M ? "forced 10M full duplex" : "auto negotiation");
}

// WARNING: onEvent is called from a separate FreeRTOS task (thread)!
void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      // The hostname must be set after the interface is started, but needs
      // to be set before DHCP, so set it from the event handler thread.
      ETH.setHostname(userData.mbusinoName);
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      eth_connected = true;
      if(ethLinkSeen == false){ // start the DHCP timeout at the first link up, link flapping must not restart it
        ethLinkSeen = true;
        timerETHmessage = millis();
      }
      timerNetworkChange = millis();
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("[ETH] %lu ms: Got IP\n", millis());
      Serial.println(ETH);
      eth_connected = true;
      gotIP = true;
      if(ethLostSince != 0){ // Ethernet came back within the grace period
        Serial.printf("[ETH] %lu ms: back after %lu ms, no restart\n", millis(), millis() - ethLostSince);
        ethLostSince = 0;
      }
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.printf("[ETH] %lu ms: Lost IP\n", millis());
      eth_connected = false;
      if(ethLostSince == 0){ // start the grace period, loop() restarts if Ethernet does not come back
        ethLostSince = millis();
        Serial.printf("[ETH] %lu ms: grace period started, restart if Ethernet does not return\n", millis());
      }
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("[ETH] Disconnected");
      eth_connected = false;
      if(gotIP == true){ // cable was unplugged, start the DHCP timeout again at the next link up
        ethLinkSeen = false;
        if(ethLostSince == 0){ // start the grace period only for an established connection
          ethLostSince = millis();
          Serial.printf("[ETH] %lu ms: grace period started, restart if Ethernet does not return\n", millis());
        }
      }
      gotIP = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default: break;
  }
}

void hwSetupEarly(){
  pinMode(LED_BUILTIN, OUTPUT); // LED on if MQTT connectet to server
  digitalWrite(LED_BUILTIN,HIGH);
  Serial.begin(115200);
}

// read the Ethernet link mode, EEPROM.begin() is already called
void hwLoadEeprom(){
  uint8_t ethLinkMode = 0;
  EEPROM.get(EEPROM_ETH_LINK_MODE, ethLinkMode);
  ethForced10M = (ethLinkMode == 1);
  ethForced10MStored = ethForced10M;
  EEPROM.get(EEPROM_ETH_MODE_SETTING, ethModeSetting);
  if(ethModeSetting > ETH_MODE_FORCED10M){ // EEPROM never written
    ethModeSetting = ETH_MODE_AUTO;
  }
  if(ethModeSetting == ETH_MODE_AUTONEG){
    ethForced10M = false;
  }else if(ethModeSetting == ETH_MODE_FORCED10M){
    ethForced10M = true;
  }
  Serial.printf("[ETH] stored link mode from EEPROM: %s, web setting: %u\n", ethForced10M ? "forced 10M full duplex" : "auto negotiation", ethModeSetting);
}

// store the settings that are not part of userData, EEPROM.begin() is already called
void hwSaveEeprom(){
  EEPROM.put(EEPROM_ETH_MODE_SETTING, ethModeSetting);
}

void hwNetworkSetup(){
  WiFi.onEvent(WiFiEvent);
  WiFi.hostname(userData.mbusinoName);
  WiFi.mode(WIFI_STA);
  WiFi.begin(userData.ssid, userData.password);

  Network.onEvent(onEvent);
  delay(100);
  setEthLinkMode();
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ, ETH_PHY_RST, ETH_PHY_SPI_HOST, ETH_PHY_SPI_SCK, ETH_PHY_SPI_MISO, ETH_PHY_SPI_MOSI);
  delay(2000);

  byte tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    if (tries++ > 5) {
      if(eth_connected == false){
        WiFi.mode(WIFI_AP);
        WiFi.softAP("MBusino Setup Portal"); //, "secret");
        apMode = true;
      }
      else{
        WiFi.mode(WIFI_OFF);
        Serial.println("WIFI set to off, no known Network and Ethernet is already connected");
      }
      break;
    }
  }

  if(eth_connected == true){
    client.setClient(ethClient);
    usedMQTTconection = 1;
    Serial.println("MQTT set connection via Ethernet");
  }
  else{
    client.setClient(wfClient);
    usedMQTTconection = 2;
    Serial.println("MQTT set connection via Wifi");
  }
}

void hwSetupLate(){
  Serial.printf("[BOOT] setup done at %lu ms\n", millis());
}

// Ethernet link mode fallback, single transport, Ethernet loss, setup AP and MQTT transport switching
void hwLoop(){
  bool ethNoIP = gotIP == false && eth_connected == true && (millis() - timerETHmessage > ETH_DHCP_TIMEOUT);
  bool ethNoLink = ethLinkSeen == false && ethModeConfirmed == false && ethModeSetting == ETH_MODE_AUTO && (millis() - timerETHbegin > ETH_LINK_TIMEOUT); // with the wrong mode some W5500 get no link at all
  if((ethNoIP == true || ethNoLink == true) && waitForRestart == false){
    if(ethModeSetting == ETH_MODE_AUTO && ethModeConfirmed == false){
      ethForced10M = !ethForced10M; // try the other link mode
    }
    Serial.printf("[ETH] %lu ms: %s, restart ETH with %s\n", millis(), ethNoLink ? "no link" : "no IP received", ethForced10M ? "forced 10M full duplex" : "auto negotiation");
    ethRestarting = millis(); // suppress transport switching until ETH is back
    timerETHmessage = millis();
    ETH.end();
    delay(400);
    setEthLinkMode();
    ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ, ETH_PHY_RST, ETH_PHY_SPI_HOST, ETH_PHY_SPI_SCK, ETH_PHY_SPI_MISO, ETH_PHY_SPI_MOSI,16);
  }

  if(gotIP == true && ethModeConfirmed == false){
    ethModeConfirmed = true;
    if(ethModeSetting == ETH_MODE_AUTO && ethForced10M != ethForced10MStored){ // remember the working link mode for the next boot
      Serial.printf("[ETH] %lu ms: link mode %s got an IP, remembering it for the next boot\n", millis(), ethForced10M ? "forced 10M full duplex" : "auto negotiation");
      uint8_t ethLinkMode = ethForced10M;
      EEPROM.begin(512);
      EEPROM.put(EEPROM_ETH_LINK_MODE, ethLinkMode);
      EEPROM.commit();
      EEPROM.end();
      ethForced10MStored = ethForced10M;
    }
  }

  // an intentional ETH restart (link mode change) is finished once an IP is back, or after the grace period at the latest
  if(ethRestarting != 0 && (gotIP == true || (millis() - ethRestarting) > ETH_LOSS_GRACE)){
    ethRestarting = 0;
  }

  //--- keep exactly one transport alive: Ethernet > Wi-Fi > setup AP -------------
  // As soon as Ethernet has an IP it is the only transport. The setup AP and a
  // possibly connected Wi-Fi STA are shut down here, otherwise the device ends up
  // with two IPs and MQTT would flip between them on every link change.
  if(gotIP == true && eth_connected == true){
    if(apMode == true){
      Serial.printf("[NET] %lu ms: Ethernet has an IP -> shutting down the setup AP\n", millis());
      dnsServer.stop();
      WiFi.softAPdisconnect(true);
      apMode = false;
    }
    if(WiFi.getMode() != WIFI_OFF){
      Serial.printf("[NET] %lu ms: Ethernet has an IP -> turning Wi-Fi off (single transport)\n", millis());
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
    }
  }

  //--- Ethernet loss handling ---------------------------------------------------
  // A working Ethernet connection may briefly drop (switch reboot, cable wiggle,
  // STP). Give it a grace period and restart only if it does not come back; after
  // the restart the boot picks the transport again (Ethernet -> Wi-Fi -> AP).
  // Without this the device could end up without any usable connection.
  if(ethLostSince != 0){
    if(eth_connected == true && gotIP == true){
      ethLostSince = 0; // safety net, the ETH_GOT_IP event normally clears it already
    }
    else if((millis() - ethLostSince) > ETH_LOSS_GRACE){
      Serial.printf("[NET] %lu ms: Ethernet gone for more than %lu ms -> restarting to re-pick the transport\n", millis(), (unsigned long)ETH_LOSS_GRACE);
      Serial.flush();
      ESP.restart();
    }
  }

  if(wifiReconnect == true && (millis() - timerWifiReconnect > 2000)){
    wifiReconnect = false;
    if(eth_connected == true){ // Ethernet is the active transport, do not bring Wi-Fi up in addition
      Serial.printf("[NET] %lu ms: Wi-Fi reconnect skipped, Ethernet is connected\n", millis());
    }
    else if(WiFi.getMode() == WIFI_OFF){ // Wi-Fi was switched off for the Ethernet transport, do not bring it back silently
      Serial.printf("[NET] %lu ms: Wi-Fi reconnect skipped, Wi-Fi is off (Ethernet transport)\n", millis());
    }
    else{
      Serial.println("try to reconnect");
      WiFi.reconnect();
    }
  }

  if((apMode == true && eth_connected == false) && millis() > 300000){
    ESP.restart();
  }

  if(apMode == true){
    dnsServer.processNextRequest();
    if((millis() - timerLedAPmode) > 500){ // blink while the setup AP is running
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      timerLedAPmode = millis();
    }
  }

  //to notice Changes in Network an assign the right connection to the MQTT client
  //Do not fall back to Wi-Fi while ETH is intentionally re-initialised (link mode change)
  //or when Wi-Fi is switched off for the Ethernet transport.
  if((eth_connected == false) && (usedMQTTconection == 1) && ethRestarting == 0 && (WiFi.getMode() != WIFI_OFF) && (millis()-timerNetworkChange) > 5000){
    client.setClient(wfClient);
    usedMQTTconection = 2;
    Serial.println("MQTT set connection via Wifi");
    reconnect();
    timerNetworkChange = millis();
  }
  if((eth_connected == true) && (usedMQTTconection == 2) && (millis()-timerNetworkChange) > 5000){
    client.setClient(ethClient);
    usedMQTTconection = 1;
    Serial.println("MQTT set connection via Ethernet");
    reconnect();
    timerNetworkChange = millis();
  }
}

// MQTT may be used: always, the transport is chosen above
bool hwMqttAllowed(){
  return true;
}

void hwReconnect(){
  Serial.println("MQTT no connection");
  if(apMode == false || gotIP == true){ // Ethernet may come up after the AP was started
    reconnect();
  }
  else{
    Serial.println(" --> AP is running");
  }
}

void hwPublishSettings(){
  client.publish(String(String(userData.mbusinoName) + "/settings/wl_Connected").c_str(), String(WL_CONNECTED).c_str());
  client.publish(String(String(userData.mbusinoName) + "/settings/apMode").c_str(), String(apMode).c_str());
  client.publish(String(String(userData.mbusinoName) + "/settings/eth_connected").c_str(), String(eth_connected).c_str());
  client.publish(String(String(userData.mbusinoName) + "/settings/ethLinkMode").c_str(), ethForced10M ? "forced 10M full duplex" : "auto negotiation");
  client.publish(String(String(userData.mbusinoName) + "/settings/wifiIP").c_str(), String(WiFi.localIP().toString()).c_str());
  client.publish(String(String(userData.mbusinoName) + "/settings/ethIP").c_str(), String(ETH.localIP().toString()).c_str());
  if(usedMQTTconection == 1){
    client.publish(String(String(userData.mbusinoName) + "/settings/MQTTconnectedVia").c_str(), "ethernet");
  }
  if(usedMQTTconection == 2){
    client.publish(String(String(userData.mbusinoName) + "/settings/MQTTconnectedVia").c_str(), "WiFi");
  }
  long rssi = (WiFi.getMode() != WIFI_OFF) ? WiFi.RSSI() : 0; // Wi-Fi is off while Ethernet is the transport, an RSSI value would be meaningless
  client.publish(String(String(userData.mbusinoName) + "/settings/RSSI").c_str(), String(rssi).c_str());
}

// runs while MQTT is connected, nothing besides M-Bus on the Nano
void hwConnectedLoop(){}

// MQTT hooks, the Nano has no additional topics
void hwMqttReconnected(){}
void hwMqttSubscribe(){}
void hwMqttCallback(char* topic, byte* payload){}
