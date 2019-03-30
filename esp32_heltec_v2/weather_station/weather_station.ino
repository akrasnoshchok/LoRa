#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8x8lib.h>
#include <CayenneLPP.h>
#include "DHTesp.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <time.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <EEPROM.h>
#include <Arduino.h>

//***************************************************************************************************
// Constants                                                                                        *
//***************************************************************************************************
#define DHTPIN 13                               // Digital pin connected to the DHT sensor
#define SEALEVELPRESSURE_HPA (1013.25)
#define CFG_eu868 1
#define DATAVALID 0xACF2AFC2                     // Pattern for data valid in EEPROM/RTC memory

//***************************************************************************************************
// Commissioning data                                                                               *
//***************************************************************************************************
static const u1_t PROGMEM
APPEUI[8] = {
0x65, 0x8A, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70
};

void os_getArtEui(u1_t *buf) {
    memcpy_P(buf, APPEUI, 8);
}

static const u1_t PROGMEM
DEVEUI[8] = {
0x6E, 0xE7, 0x5C, 0x40, 0x7A, 0x78, 0x97, 0x00
};

void os_getDevEui(u1_t *buf) {
    memcpy_P(buf, DEVEUI, 8);
}

static const u1_t PROGMEM
APPKEY[16] = {
0xFD, 0xDD, 0xFC, 0x28, 0x2B, 0x7F, 0x9F, 0xCE, 0xC5, 0xC8, 0x3B, 0xF7, 0x52, 0x4D, 0x2A, 0xB6
};

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

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

u1_t NWKSKEY[16];                               // LoRaWAN NwkSKey, network session key.
u1_t APPSKEY[16];                               // LoRaWAN AppSKey, application session key.
u4_t DEVADDR;                                   // LoraWAN devaddr, end node device address
bool sleepreq = false;
savdata_t savdata;
static osjob_t sendjob;

const unsigned TX_INTERVAL = 600;

// Pin mapping
const lmic_pinmap lmic_pins = {
        .nss = 18,
        .rxtx = LMIC_UNUSED_PIN,
        .rst = 14,
        .dio = {26, 34, 35},
};

HardwareSerial SerialGPS(2);
DHTesp dht;
TempAndHumidity dhtData;
Adafruit_BMP280 bmp280;
TinyGPSPlus gps;
CayenneLPP lpp(51);
bool OTAA = true;

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
    u8x8.clearLine(7);
    u8x8.setCursor(0, 3);
    u8x8.printf("TIME %02d:%02d:%02d", gps.time.hour() + 2, gps.time.minute(), gps.time.second());
    Serial.print(": ");
    switch (ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            u8x8.drawString(0, 7, "EV_SCAN_TIMEOUT");
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            u8x8.drawString(0, 7, "EV_BEACON_FOUND");
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            u8x8.drawString(0, 7, "EV_BEACON_MISSED");
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            u8x8.drawString(0, 7, "EV_BEACON_TRACKED");
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            u8x8.drawString(0, 7, "EV_JOINING");
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            u8x8.drawString(0, 7, "EV_JOINED ");

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            u8x8.drawString(0, 7, "EV_RFUI");
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            u8x8.drawString(0, 7, "EV_JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            u8x8.drawString(0, 7, "EV_REJOIN_FAILED");
            //break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            u8x8.drawString(0, 7, "EV_TXCOMPLETE");
            digitalWrite(BUILTIN_LED, LOW);
            if (LMIC.txrxFlags & TXRX_ACK) {
                Serial.println(F("Received ack"));
                u8x8.clearLine(7);
                u8x8.drawString(0, 7, "Received ACK");
            }
            if (LMIC.dataLen) {
                Serial.println(F("Received "));
                u8x8.drawString(0, 6, "RX ");
                Serial.println(LMIC.dataLen);
                u8x8.clearLine(6);
                u8x8.setCursor(4, 6);
                u8x8.printf("%i bytes", LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
                u8x8.clearLine(7);
                u8x8.setCursor(0, 7);
                u8x8.printf("RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            sleepreq = true;
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            u8x8.drawString(0, 7, "EV_LOST_TSYNC");
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            u8x8.drawString(0, 7, "EV_RESET");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            u8x8.drawString(0, 7, "EV_RXCOMPLETE");
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            u8x8.drawString(0, 7, "EV_LINK_DEAD");
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            u8x8.drawString(0, 7, "EV_LINK_ALIVE");
            break;
        default:
            Serial.println(F("Unknown event"));
            u8x8.setCursor(0, 7);
            u8x8.printf("UNKNOWN EVENT %d", ev);
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
        u8x8.drawString(0, 7, "OP_TXRXPEND, not sent");
    } else {
        // Prepare upstream data transmission at the next possible time.
        Serial.println("Prepare upstream data transmission");

        getTemperature();

        debugTemperature();
        debugBmp280();
        debugGps();

        lpp.reset();

        lpp.addTemperature(1, dhtData.temperature);
        lpp.addRelativeHumidity(2, dhtData.humidity);
        lpp.addBarometricPressure(3, bmp280.readPressure() / 100);
        lpp.addAnalogInput(4, analogRead(0));
        if (gps.location.lat() != 0 && gps.location.lng() != 0 && gps.altitude.meters() != 0) {
            lpp.addGPS(5, gps.location.lat(), gps.location.lng(), gps.altitude.meters());
        }

        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);

        u8x8.setCursor(0, 5);
        u8x8.printf("msg queued %ib", lpp.getSize());
        Serial.println(F("Packet queued"));
        u8x8.drawString(0, 7, "PACKET QUEUED");
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
//                               G E T   T E M P E R A T U R E                                      *
//***************************************************************************************************
bool getTemperature() {
    // Reading temperature for humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    dhtData = dht.getTempAndHumidity();
    // Check if any reads failed and exit early (to try again).
    if (dht.getStatus() != 0) {
        Serial.println("DHT22 error status: " + String(dht.getStatusString()));
        return false;
    }

    return true;
}


//***************************************************************************************************
//                                  G E T   G P S   D A T A                                         *
//***************************************************************************************************
void getGPSdata() {
    while (!gps.location.isUpdated() || gps.location.age() > 10000) {
        if (SerialGPS.available()) {
            gps.encode(SerialGPS.read());
        }
    }
}

//***************************************************************************************************
//                                              D E B U G                                           *
//***************************************************************************************************

void debugTemperature() {
    float heatIndex = dht.computeHeatIndex(dhtData.temperature, dhtData.humidity);
    float dewPoint = dht.computeDewPoint(dhtData.temperature, dhtData.humidity);
    Serial.println(
            " T:" + String(dhtData.temperature) + " H:" + String(dhtData.humidity) + " I:" + String(heatIndex) + " D:" +
            String(dewPoint));
}

void debugBmp280() {
    Serial.print("----- BMP 280 -----\n");
    Serial.print("Temperature = ");
    Serial.print(bmp280.readTemperature());
    Serial.println(" *C");
    Serial.print("Pressure = ");
    Serial.print(bmp280.readPressure() / 100); // 100 Pa = 1 millibar
    Serial.println(" mb");
    Serial.print("Approx altitude = ");
    Serial.print(bmp280.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
}

void debugGps() {
    Serial.print("----- GPS -----\n");

    if (gps.location.isValid()) {
        Serial.print(F("LOCATION:  "));
        Serial.print(F(" Lat="));
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(" Long="));
        Serial.println(gps.location.lng(), 6);
    }

    if (gps.date.isValid()) {
        Serial.print(F("DATE:  "));
        Serial.print(gps.date.day());
        Serial.print(F("."));
        Serial.print(gps.date.month());
        Serial.print(F("."));
        Serial.println(gps.date.year());
    }

    if (gps.time.isValid()) {
        Serial.print(F("TIME:  "));
        Serial.print(gps.time.hour() + 2);
        Serial.print(F(":"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        Serial.println(gps.time.second());
    }

    if (gps.time.isValid()) {
        Serial.print(F("ALTITUDE:  "));
        Serial.print(F(" Meters="));
        Serial.print(gps.altitude.meters());
    }

    Serial.print(F("SATELLITES:  "));
    Serial.println(gps.satellites.value());
}


//***************************************************************************************************
//                                              S E T U P                                           *
//***************************************************************************************************
void setup() {

    Serial.begin(115200);
    SerialGPS.begin(115200, SERIAL_8N1, 25, 23);
    EEPROM.begin(512);

    getGPSdata();

    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.drawString(0, 1, "LoRaWAN Academy");

    dht.setup(DHTPIN, DHTesp::DHT22);
    Serial.println("DHT initiated");

    if (!bmp280.begin(0x76))
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
    else
        Serial.println("BMP280 initiated");

    SPI.begin(5, 19, 27);

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
