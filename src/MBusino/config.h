// MBusino variant selection
//
// The hardware is detected from the board selected in the Arduino IDE:
//   LOLIN(WEMOS) D1 mini (ESP8266) or LOLIN S2 Mini (ESP32-S2) -> MBusino shield with OneWire/BME280 sensors
//   MakerGO ESP32 C3 SuperMini                                 -> MBusino Nano with W5500 Ethernet
//
// The number of M-Bus slaves has to be chosen here: 1 for MBusino / MBusino Nano,
// 5 for MBusino5S / MBusino Nano5S. tools/build-all.sh sets it with -DMBUSINO_SLAVES=...

#ifndef MBUSINO_SLAVES
#define MBUSINO_SLAVES 1 // 1 or 5
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3)
#define MBUSINO_ETH     // W5500 Ethernet, M-Bus only
#elif defined(ESP8266) || defined(CONFIG_IDF_TARGET_ESP32S2)
#define MBUSINO_SENSORS // shield with OneWire and BME280
#else
#error "unsupported board, use LOLIN(WEMOS) D1 mini, LOLIN S2 Mini or MakerGO ESP32 C3 SuperMini"
#endif

#if MBUSINO_SLAVES != 1 && MBUSINO_SLAVES != 5
#error "MBUSINO_SLAVES must be 1 or 5"
#endif

// debug output on the serial monitor; on the ESP8266 Serial is the M-Bus UART, so nothing is printed there
#if defined(ESP8266)
#define LOG(...)
#else
#define LOG(...) Serial.printf(__VA_ARGS__)
#endif
