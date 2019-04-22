/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with static payload,
 * using frequency and encryption settings matching those of
 * the (early prototype version of) The Things Network.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * ToDo:
 * - set NWKSKEY (value from staging.thethingsnetwork.com)
 * - set APPKSKEY (value from staging.thethingsnetwork.com)
 * - set DEVADDR (value from staging.thethingsnetwork.com)
 * - optionally comment #define DEBUG
 * - optionally comment #define SLEEP
 * - set TX_INTERVAL in seconds
 * - change mydata to another (small) static text
 *
 *******************************************************************************/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// LoRaWAN NwkSKey, your network session key, 16 bytes (from staging.thethingsnetwork.org)
static const PROGMEM u1_t NWKSKEY[16] = { 0x77, 0x87, 0x5C, 0x42, 0x05, 0x91, 0xA5, 0xEF, 0x23, 0x7B, 0xA0, 0xC1, 0x4B, 0xB2, 0xE3, 0xB1 };

// LoRaWAN AppSKey, application session key, 16 bytes  (from staging.thethingsnetwork.org)
static const u1_t PROGMEM APPSKEY[16] = { 0x59, 0x12, 0x03, 0xF1, 0x17, 0xD1, 0x34, 0x85, 0xF9, 0x7C, 0x6B, 0x83, 0xC8, 0x06, 0xE0, 0xD9 };

// LoRaWAN end-device address (DevAddr), ie 0x91B375AC  (from staging.thethingsnetwork.org)
static const u4_t DEVADDR = 0x2601149E ; // <-- Change this address for every node!


// show debug statements; comment next line to disable debug statements
#define DEBUG
// use low power sleep; comment next line to not use low power sleep
//#define SLEEP

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// static text, you can replace T
static uint8_t mydata[] = "T";

#ifdef SLEEP
#include "LowPower.h"
bool next = false;
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
        .nss = 6,
        .rxtx = LMIC_UNUSED_PIN,
        .rst = 5,
        .dio = {2, 3, 4},
};

void onEvent (ev_t ev) {
#ifdef DEBUG
    Serial.println(F("Enter onEvent"));
#endif

    switch(ev) {
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
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
#ifndef SLEEP
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
#else
            next = true;
#endif

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
#ifdef DEBUG
    Serial.println(F("Leave onEvent"));
#endif

}

void do_send(osjob_t* j){

#ifdef DEBUG
    Serial.println(F("Enter do_send"));
#endif

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
#ifdef DEBUG
    Serial.println(F("Leave do_send"));
#endif

}

void setup() {
    Serial.begin(9600);
    Serial.println(F("Enter setup"));

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);

#ifdef DEBUG
    Serial.println(F("Leave setup"));
#endif
}

void loop() {

#ifndef SLEEP

    os_runloop_once();

#else
    extern volatile unsigned long timer0_overflow_count;

    if (next == false) {

        os_runloop_once();

    } else {

        int sleepcycles = TX_INTERVAL / 8;  // calculate the number of sleepcycles (8s) given the TX_INTERVAL
#ifdef DEBUG
        Serial.print(F("Enter sleeping for "));
        Serial.print(sleepcycles);
        Serial.println(F(" cycles of 8 seconds"));
#endif
        Serial.flush(); // give the serial print chance to complete
        for (int i=0; i<sleepcycles; i++) {
            // Enter power down state for 8 s with ADC and BOD module disabled
            LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
            //LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);

            // LMIC uses micros() to keep track of the duty cycle, so
            // hack timer0_overflow for a rude adjustment:
            cli();
            timer0_overflow_count+= 8 * 64 * clockCyclesPerMicrosecond();
            sei();
        }
#ifdef DEBUG
        Serial.println(F("Sleep complete"));
#endif
        next = false;
        // Start job
        do_send(&sendjob);
    }

#endif

}

