/**
 * Copyright (c) 2015 Alex Tang
 */
#if ARDUINO>=100
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif

#include "SevconGen4.h"

SevconGen4::SevconGen4() {
    // init
}

#define CANSPEED_1000 0
void SevconGen4::init() {
    canOpenLite = CanOpenLite();
    initSuccess = mcp2515_init(CANSPEED_1000);
    idsToFetch[0] = &throttleValueOD;
    idsToFetch[2] = &batteryVoltageOD;
    idsToFetch[3] = &batteryCurrentOD;
    idsToFetch[4] = &motorVoltageOD;
    idsToFetch[5] = &motorCurrentOD;
    idsToFetch[6] = &heatsinkTempOD;
    idsToFetch[7] = &motorTempControllerOD;
    idsToFetch[8] = &speedOD;
    idsToFetch[9] = &rpmOD;
    idsToFetch[10] = NULL;
}

bool SevconGen4::processData() {
        // Send CANOpen SDO requests for data if we haven't received or sent requests for it in a while.
        // wait until the system has been up for at least 3 seconds before starting.
    unsigned long currentMillis = millis();
    if ( currentMillis > 3000 ) {
        uint8_t idCount;
        for ( idCount = 0; idsToFetch[idCount] != NULL; idCount++ ) {
            ObjectDictionaryEntry *entry = idsToFetch[idCount];
                // If we haven't sent a request for or received data for the entry in the last 600ms, 
                // send a new request for the data. 
            if ( ( ( millis() - entry->timeReceived ) > 600 ) && ( ( millis() - entry->timeRequested ) > 600 ) ) {
                entry->timeRequested = millis();
                request.id = 0x601;
                request.header.rtr = 0;
                request.header.length = 8;
                sdoMsg.type = 2;
                sdoMsg.length = 0;
                sdoMsg.index = entry->index;
                sdoMsg.subIndex = entry->subIndex;
                sdoMsg.data = 0x0;
                canOpenLite.sdoMessageToBuffer ( &sdoMsg, request.data );
                mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
                mcp2515_send_message(&request);
            }
        }
    }

        // Read CANOpen SDO responses
    while (mcp2515_check_message()) {
        memset ( &response, 0, sizeof(tCAN));
        if (mcp2515_get_message(&response)) {
            if ( response.id == 0x581 ) {
                canOpenLite.sdoMessageFromBuffer ( &sdoMsg, response.data );
                for ( int i = 0; idsToFetch[i] != NULL ; i++ ) {
                    ObjectDictionaryEntry *entry = idsToFetch[i];
                    if ( ( sdoMsg.index == entry->index ) && ( sdoMsg.subIndex == entry->subIndex ) ) {
                        //currentMillis = millis();
                        entry->value = sdoMsg.data * entry->scalingFactor;
                        entry->timeReceived = millis();
                        break;
                    }
                }
            }
        }
    }
}
