#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <time.h>
#include <HardwareSerial.h>
#include <EEPROM.h>
#include <Arduino.h>

#include "esp_bt_device.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

//***************************************************************************************************
// Constants                                                                                        *
//***************************************************************************************************
#define CFG_eu868 1
#define DATAVALID 0xACF2AFC2                     // Pattern for data valid in EEPROM/RTC memory


//***************************************************************************************************
// Commissioning data                                                                               *
//***************************************************************************************************
static const u1_t PROGMEM
APPEUI[8] = { 0x0F, 0xCF, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };

void os_getArtEui(u1_t *buf) {
  memcpy_P(buf, APPEUI, 8);
}

static const u1_t PROGMEM
DEVEUI[8] = { 0x55, 0x4C, 0x12, 0x8B, 0x45, 0x90, 0xA2, 0x00 };

void os_getDevEui(u1_t *buf) {
  memcpy_P(buf, DEVEUI, 8);
}

static const u1_t PROGMEM
APPKEY[16] = { 0x8C, 0x1D, 0x5C, 0xAA, 0x0C, 0x8A, 0x29, 0xF4, 0x81, 0x9C, 0xD7, 0xE8, 0x83, 0x2A, 0x3E, 0x39 };

void os_getDevKey(u1_t *buf) {
  memcpy_P(buf, APPKEY, 16);
}

//***************************************************************************************************
// Global data                                                                                      *
//***************************************************************************************************

struct savdata_t                                 // Structure of data to be saved over reset
{
  uint32_t dataValid;                          // DATAVALID if valid data (joined)
  uint8_t devaddr[4];                          // Device address after join
  uint8_t nwkKey[16];                          // Network session key after join
  uint8_t artKey[16];                          // Aplication session key after join
  uint32_t seqnoUp;                            // Sequence number
};

u1_t NWKSKEY[16];                               // LoRaWAN NwkSKey, network session key.
u1_t APPSKEY[16];                               // LoRaWAN AppSKey, application session key.
u4_t DEVADDR;                                   // LoraWAN devaddr, end node device address
bool sleepreq = false;
savdata_t savdata;
static osjob_t sendjob;

const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 32, 33},
};

bool OTAA = true;

BLEScan *pBLEScan;

String known_ble_names[] = { "RDL51822" };

typedef struct device {
  esp_bd_addr_t mac;
  int rssi;
} device;

unsigned char buffer[ESP_BD_ADDR_LEN*3];

device devices[3] = {
  {"", INT_MIN},
  {"", INT_MIN},
  {"", INT_MIN}
};

//***************************************************************************************************
//                                      M E M    D M P                                              *
//***************************************************************************************************
void memdmp(const char *header, uint8_t *p, uint16_t len) {
  uint16_t i;                                                        // Loop control

  Serial.print(header);                                           // Show header
  for (i = 0; i < len; i++) {
    if ((i & 0x0F) == 0)                                          // Continue opn next line?
    {
      if (i > 0)                                                    // Yes, continuation line?
      {
        Serial.printf("\n");                                      // Yes, print it
      }
      Serial.printf("%04X: ", i);                                 // Print index
    }
    Serial.printf("%02X ", *p++);                                 // Print one data byte
  }
  Serial.println();
}

//***************************************************************************************************
//                                  S A V E    T O    E E P R O M                                   *
//***************************************************************************************************
// Save data in EEPROM memory.                                                                      *
//***************************************************************************************************
void saveToEEPROM() {
  uint16_t eaddr;                                  // Address in EEPROM
  uint8_t *p;                                      // Points into savdata

  Serial.printf("Save data to EEPROM memory\n");
  memcpy(savdata.devaddr, &LMIC.devaddr, 4);           // Fill struct to save
  memcpy(savdata.nwkKey, LMIC.nwkKey, 16);
  memcpy(savdata.artKey, LMIC.artKey, 16);
  savdata.seqnoUp = LMIC.seqnoUp;
  savdata.dataValid = DATAVALID;
  memdmp("devaddr:", savdata.devaddr, 4);
  memdmp("nwkKey:", savdata.nwkKey, 16);
  memdmp("artKey:", savdata.artKey, 16);
  Serial.printf("SeqnoUp is %d\n", savdata.seqnoUp);
  Serial.println("Saving to EEPROM");
  p = (uint8_t * ) & savdata;                               // set target pointer
  for (eaddr = 0; eaddr < sizeof(savdata); eaddr++) {
    EEPROM.write(eaddr, *p++);                       // Write to EEPROM
  }
  EEPROM.commit();                                      // Commit data to EEPROM
}

//***************************************************************************************************
//                                        O N   E V E N T                                           *
//***************************************************************************************************

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      //break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      digitalWrite(BUILTIN_LED, LOW);
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received ack"));
      }
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      sleepreq = true;
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

//***************************************************************************************************
//                                            D O _ S E N D                                         *
//***************************************************************************************************
// Send a message to TTN.                                                                           *
//***************************************************************************************************

void do_send(osjob_t *j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    Serial.println("Prepare upstream data transmission");

    for (int i = 0; i < sizeof(devices) / sizeof(devices[0]); i++) {
      devices[i] = {{0}, INT_MIN};
    }

    pBLEScan->start(5);

    int devicesCount = sizeof(devices) / sizeof(devices[0]);

    int bufferOffset = 0;
    for (int i = 0; i < devicesCount; i++,bufferOffset+=sizeof(devices[i].mac)) {
      memcpy(buffer + bufferOffset, devices[i].mac, sizeof(devices[i].mac));
    }

    LMIC_setTxData2(1, buffer, sizeof(buffer), 0);

    Serial.println(F("Packet queued"));
    digitalWrite(BUILTIN_LED, HIGH);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

//***************************************************************************************************
//                              R E T R I E V E   K E Y S                                           *
//***************************************************************************************************
// Try to retrieve the keys, seqnr from EEPROM memory.                                              *
//***************************************************************************************************
void retrieveKeys() {
  uint16_t eaddr;                                          // Address in EEPROM
  uint8_t *p;                                              // Pointer into savdata

  p = (uint8_t * ) & savdata;
  for (eaddr = 0; eaddr < sizeof(savdata); eaddr++) {
    *p++ = EEPROM.read(eaddr);                        // Move one byte to savdata
  }

  if (savdata.dataValid == DATAVALID)                     // DATA in RTC or EEPROM memory valid?
  {
    Serial.printf("Valid data in NVS\n");               // Yes, show
    memdmp("devaddr is:",
           savdata.devaddr, 4);
    memdmp("nwksKey is:",
           savdata.nwkKey, 16);
    memdmp("appsKey is:",
           savdata.artKey, 16);
    Serial.printf("seqnr is %d\n", savdata.seqnoUp);
    memcpy((uint8_t * ) & DEVADDR,
           savdata.devaddr, sizeof(DEVADDR));          // LoraWAN DEVADDR, end node device address
    memcpy(NWKSKEY,
           savdata.nwkKey, sizeof(NWKSKEY));          // LoRaWAN NwkSKey, network session key.
    memcpy(APPSKEY,
           savdata.artKey, sizeof(APPSKEY));          // LoRaWAN AppSKey, application session key.
    OTAA = false;                                         // Do not use OTAA
  } else {
    Serial.printf("No saved data, using OTAA\n");
  }
}

//***************************************************************************************************
//                                              B L E                                               *
//***************************************************************************************************

double getDistance(int rssi, int txPower) {
  /*
     From Pravin Uttarwar

     https://www.quora.com/How-do-I-calculate-distance-in-meters-km-yards-from-rssi-values-in-dBm-of-BLE-in-android

     RSSI = TxPower - 10 * n * lg(d)
     n = 2 (in free space)

     d = 10 ^ ((TxPower - RSSI) / (10 * n))
  */

  return pow((double)10, ((double) txPower - rssi) / (10 * 2));
}

void bleDebug(BLEAdvertisedDevice advertisedDevice) {
  Serial.print("BLE Advertised Device found: ");
  Serial.println(advertisedDevice.toString().c_str());
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {

      if (!advertisedDevice.haveName()) return;

      for (int i = 0; i < sizeof(known_ble_names) / sizeof(known_ble_names[0]); i++) {
        if (strcmp(advertisedDevice.getName().c_str(), known_ble_names[i].c_str()) == 0 && advertisedDevice.haveRSSI()) {
          bleDebug(advertisedDevice);
          //float distance = getDistance(advertisedDevice.getRSSI(), -55);
          //Serial.println(distance);

          int devicesCount = sizeof(devices) / sizeof(devices[0]);

          for (int k = 0; k < devicesCount; k++) {
            if (advertisedDevice.getRSSI() > devices[k].rssi) {
              if (k < devicesCount - 1) {
                for (int l = devicesCount - 1; l > k; l--) {
                  devices[l] = devices[l - 1];
                }
              }

              memcpy( devices[k].mac, advertisedDevice.getAddress().getNative(), ESP_BD_ADDR_LEN );
              devices[k].rssi = advertisedDevice.getRSSI();

              for (int j = 0; j < devicesCount; j++) {
                Serial.printf("!!! %s, %d\n", devices[j].mac, devices[j].rssi);
              }
              break;
            }
          }
        }
      }
    }
};

void ble_init() {
  BLEDevice::init("");

  // put your main code here, to run repeatedly:
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(0x50);
  pBLEScan->setWindow(0x30);
}


//***************************************************************************************************
//                                              S E T U P                                           *
//***************************************************************************************************
void setup() {

  Serial.begin(115200);
  digitalWrite(BUILTIN_LED, HIGH);
  //SerialGPS.begin(115200, SERIAL_8N1, 12, 15);
  EEPROM.begin(512);

  ble_init();

  retrieveKeys();

  os_init();
  LMIC_reset();

  if (OTAA) {
    Serial.println("Start joining");
    LMIC_startJoining();
  } else {
    memdmp("Set Session, DEVADDR:", (uint8_t * ) & DEVADDR, 4);
    memdmp("NWKSKEY:", NWKSKEY, 16);
    memdmp("APPSKEY:", APPSKEY, 16);
    Serial.printf("Seqnr set to %d\n", savdata.seqnoUp);
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
    LMIC.seqnoUp = savdata.seqnoUp;                      // Correction counter
    do_send(&sendjob);
  }


  //pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
}

//***************************************************************************************************
//                                        L O O P                                                   *
//***************************************************************************************************
void loop() {

  uint32_t sleeptime;                                        // Time to sleep to next sample
  uint32_t tnow;                                             // Current runtime in microseconds

  os_runloop_once();

  if (sleepreq)                                             // Time to go to sleep?
  {
    tnow = millis() * 1000;                                  // Current runtime in microseconds
    saveToEEPROM();                                             // Save to RTC memory
    sleeptime = TX_INTERVAL * 1000000;                       // Sleeptime in microseconds
    if (sleeptime > tnow)                                   // Prevent negative sleeptime
    {
      sleeptime = sleeptime - tnow;                          // Correction for duration of program
    }
    Serial.printf("Going to sleep for %d seconds....", TX_INTERVAL);
    ESP.deepSleep(sleeptime);
  }
}
