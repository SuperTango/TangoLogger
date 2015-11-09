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
    MotorController::init();
    //Serial.print ( "Initing KLS" );
    controllerStream = stream2;
    memset ( receiveBuffer, 0, KLS8080I_RECEIVE_BUFSIZE );
    receiveBufferIndex = 0;
}

bool KellyKLS_Serial::processData() {
    if ( ( millis() - lastControllerRequestTime ) >= 300 ) {
        lastControllerRequestTime = millis();
        sendBuffer[0] = requestType;
        sendBuffer[1] = 0x00;
        sendBuffer[2] = requestType;
        if ( requestType == REQUEST_TYPE_3A ) {
            requestType = REQUEST_TYPE_3B;
        } else {
            requestType = REQUEST_TYPE_3A;
        }
        controllerStream->write ( sendBuffer, 3 );
    }

    uint8_t byteRead;
    while ( ( controllerStream->available() > 0 ) && ( receiveBufferIndex < KLS8080I_RECEIVE_BUFSIZE ) ) {
        byteRead = (uint8_t)controllerStream->read() & 0xFF;
        if ( ( receiveBufferIndex == 0 ) && ( byteRead != REQUEST_TYPE_3A ) && ( byteRead != REQUEST_TYPE_3B ) ) {
            continue;
        }
        if ( ( receiveBufferIndex == 1 ) && ( byteRead != 0x10 ) ) {
            receiveBufferIndex = 0;
            continue;
        }
        if ( receiveBufferIndex < KLS8080I_RECEIVE_BUFSIZE) {
            receiveBuffer[receiveBufferIndex] = byteRead;
            receiveBufferIndex++;
        }
    }

    if ( receiveBufferIndex == KLS8080I_RECEIVE_BUFSIZE ) {
        if ( validateChecksum() ) {
            if ( receiveBuffer[0] == REQUEST_TYPE_3A ) {
                throttlePercent = ( receiveBuffer[2] - 1 ) / 255.0;
                direction = ( receiveBuffer[7] ) ? -1 : 1;
                batteryVoltage = receiveBuffer[11] * 1.0;
                controllerTemp = receiveBuffer[13] * 1.0;
                last3APacketReceivedMillis = millis();
            } else {
                rpm = receiveBuffer[4] << 8 | receiveBuffer[5];
                motorCurrent = ( receiveBuffer[6] << 8 | receiveBuffer[7] ) * 0.1;
                last3BPacketReceivedMillis = millis();
            }
            receiveBufferIndex = 0;
            memset ( receiveBuffer, 0, KLS8080I_RECEIVE_BUFSIZE );
            return true;
        } else {
            receiveBufferIndex = 0;
            memset ( receiveBuffer, 0, KLS8080I_RECEIVE_BUFSIZE );
        }
    }
    return false;
}

bool KellyKLS_Serial::validateChecksum() {
    uint16_t sum = 0;
    for ( int i = 0; i < KLS8080I_RECEIVE_BUFSIZE - 1; i++ ) {
        sum += receiveBuffer[i];
    }
    return ( ( sum & 0xFF ) == receiveBuffer[KLS8080I_RECEIVE_BUFSIZE - 1] );
}

/*
void KellyKLS_Serial::dumpReceiveBuffer() {
    Serial.print ( receiveBufferIndex );
    Serial.print ( ": " );
    for ( int i = 0; i < KLS8080I_RECEIVE_BUFSIZE; i++ ) {
        Serial.print ( receiveBuffer[i], HEX );
        Serial.print ( " " );
    }
    Serial.println();
}
*/
