#include "Arduino.h"
#include "CanOpenLite.h"

CanOpenLite::CanOpenLite() {
}

void CanOpenLite::sdoMessageFromBuffer ( SdoMessage *sdoMessage, uint8_t *buffer ) {
    memset ( sdoMessage, 0, sizeof ( SdoMessage ) );
    sdoMessage->type = buffer[0] >> 5;
    if ( buffer[0] & 0x01 ) {
        sdoMessage->length = 4 - ( buffer[0] >> 2 & 0x03 );
    } else {
        sdoMessage->length = 0;
    }
    sdoMessage->index = buffer[2] << 8 | buffer[1];
    sdoMessage->subIndex = buffer[3];
    memcpy ( &sdoMessage->data, &buffer[4], sdoMessage->length );
}

void CanOpenLite::sdoMessageToBuffer ( SdoMessage *sdoMessage, uint8_t *buffer ) {
    memset ( buffer, 0, 8 );
    buffer[0] = sdoMessage->type << 5 & 0x70;
    if ( sdoMessage->length ) {
        buffer[0] |= ( 4 - sdoMessage->length ) << 2 & 0x0C;
        buffer[0] |= 0x03;
    }
    buffer[1] = sdoMessage->index & 0xFF; /* index LSB */
    buffer[2] = sdoMessage->index >> 8 & 0xFF; /* index MSB */
    buffer[3] = sdoMessage->subIndex;
    memcpy ( &buffer[4], &sdoMessage->data, sdoMessage->length );
}
