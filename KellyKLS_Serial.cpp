/**
 * Copyright (c) 2015 Alex Tang
 */
#if ARDUINO>=100
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif

#include "KellyKLS_Serial.h"

KellyKLS_Serial::KellyKLS_Serial() {
}

void KellyKLS_Serial::init ( Stream *stream2 ) {
    controllerStream = stream2;
}

bool KellyKLS_Serial::readData() {
    if ( ( millis() - lastControllerRequestTime ) >= 300 ) {
        lastControllerRequestTime = millis();
        //Serial.print ( lastControllerRequestTime, DEC );
        //Serial.println (": About to make request" );
        memset ( controllerBuffer, 0, KLS8080I_LOGDATA_BUFSIZE );
        controllerBuffer[0] = requestType;
        controllerBuffer[2] = requestType;
        if ( requestType == REQUEST_TYPE_3A ) {
            requestType = REQUEST_TYPE_3B;
        } else {
            requestType = REQUEST_TYPE_3A;
        }
        controllerStream->write ( controllerBuffer, 3 );
        controllerBufferIndex = 0;
        lastPrintedControllerBufferIndex = 0;
        memset ( controllerBuffer, 0, KLS8080I_LOGDATA_BUFSIZE );
    }

    //Serial.println ( controllerStream->available(), DEC );
    uint8_t byteRead;
    while ( ( controllerStream->available() > 0 ) && ( controllerBufferIndex < KLS8080I_LOGDATA_BUFSIZE ) ) {
        byteRead = (uint8_t)controllerStream->read() & 0xFF;
        if ( controllerBufferIndex < KLS8080I_LOGDATA_BUFSIZE) {
            controllerBuffer[controllerBufferIndex] = byteRead;
            controllerBufferIndex++;
        }
    }
    if ( controllerBufferIndex > 0 && controllerBufferIndex > lastPrintedControllerBufferIndex ) {
        //Serial.print ( millis(), DEC );
        //Serial.print (": got data.  current index: " );
        //Serial.println ( controllerBufferIndex, DEC );
        lastPrintedControllerBufferIndex = controllerBufferIndex;
    }

    if ( controllerBufferIndex == KLS8080I_LOGDATA_BUFSIZE ) {
        if ( validateChecksum() ) {
            if ( controllerBuffer[0] == REQUEST_TYPE_3A ) {
                throttlePercent = ( controllerBuffer[2] - 1 ) / 255.0;
                reverseSwitch = controllerBuffer[7];
                batteryVoltage = controllerBuffer[11] * 1.0;
                controllerTemp = controllerBuffer[13] * 1.0;
                last3APacketReceivedMillis = millis();
            } else {
                rpm = controllerBuffer[4] << 8 | controllerBuffer[5];
                motorCurrent = ( controllerBuffer[6] << 8 | controllerBuffer[7] ) * 0.1;
                last3BPacketReceivedMillis = millis();
            }
            controllerBufferIndex = 0;
        }
    }
}
bool KellyKLS_Serial::validateChecksum() {
    uint16_t sum = 0;
    for ( int i = 0; i < KLS8080I_LOGDATA_BUFSIZE - 1; i++ ) {
        sum += controllerBuffer[i];
    }
    return ( ( sum & 0xFF ) == controllerBuffer[KLS8080I_LOGDATA_BUFSIZE - 1] );
}
