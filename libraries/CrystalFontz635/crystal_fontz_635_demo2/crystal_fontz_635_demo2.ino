/**
 * Copyright (c) 2012 Alex Tang
 */
#include <PString.h>
#include <CrystalFontz635.h>

CrystalFontz635 crystalFontz635;
Packet packet;
char data[21];
char tmp[7];
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    memset ( &packet, 0, sizeof ( packet ) );
    //lcdSerial.begin(115200);
    crystalFontz635.init ( &Serial1 );
    crystalFontz635.clearLCD ( true );
    //updateLED = true;
    if ( ! crystalFontz635.getHardwareFirmwareVersion( (char *)data ) ) {
        Serial.println ( "getHardwareFirmwareVersion failed to return any data" );
    } else {
        Serial.println ( data );
        memcpy ( tmp, data, 6 );
        tmp[6] = NULL;
        Serial.print ( "Model: " );
        Serial.println ( (char *)tmp );
        crystalFontz635.printAt ( 0, 0, "Model: " );
        crystalFontz635.printAt ( 0, 7, (char *)tmp );

        memcpy ( tmp, (char *)&data[8], 3 );
        tmp[3] = NULL;
        Serial.print ( "HW: " );
        Serial.println ( (char *)tmp );
        crystalFontz635.printAt ( 1, 0, "HW Version: " );
        crystalFontz635.printAt ( 1, 12, (char *)tmp );

        memcpy ( tmp, (char *)&data[13], 3 );
        tmp[3] = NULL;
        Serial.print ( "FW: " );
        Serial.println ( (char *)tmp );
        crystalFontz635.printAt ( 2, 0, "FW Version: " );
        crystalFontz635.printAt ( 2, 12, (char *)tmp );
    }

    crystalFontz635.getSerial( (char *)data );
    Serial.println ( data );
    crystalFontz635.printAt ( 3, 0, "S: " );
    crystalFontz635.printAt ( 3, 3, (char *)data );

    if ( crystalFontz635.readUserFlash ( (int8_t *)data ) ) {
        Serial.print ( "user flash: " );
        for ( int i = 0; i < 16; i++ ) {
            Serial.println ( data[i], DEC );
        }
    }

    for ( int i = 0; i < 16; i ++ ) {
        data[i] = i * i;
    }
    if ( crystalFontz635.writeUserFlash ( (int8_t *)data ) ) {
        if ( crystalFontz635.readUserFlash ( (int8_t *)data ) ) {
            Serial.print ( "user flash: " );
            for ( int i = 0; i < 16; i++ ) {
                Serial.println ( data[i], DEC );
            }
        }
    }
    
    Serial.println ( "Hello" );
}

void loop() {
    delay ( 500000 );
}
