    /*
     * functional defines/undefs
     */
#define MOTOR_THERMISTOR

#include <SdFat.h>
#include <SdFatUtil.h>
#include <TinyGPS.h>
#include "mcp2515.h"
#include "CanOpenLite.h"
#include "EEPROM.h"
#include <PString.h>
#include "CrystalFontz635.h"

    // Wow, weird bug.  if I don't put the first function declaration here, the arduino IDE 
    // auto generator for function declaration fails.  weird.
void printString_P ( Print &stream, int index );

#define CANSPEED_1000 0
#define MAX_BUFSIZE 30
#define TIMEZONEOFFSET -8
#define METERSTOMILES 0.000621371192
    // this is to compenstate for the fact that the previous physical arduino board i was using had a wildly wrong clock counter.  1 "real" second == about 750 millis().  ?!?!
    // the current board is much better.
#define ARDUINO_MILLIS_COMPENSATION_FACTOR 1.001883884
#define MILLISPERHOUR 3600000
#define GPSRATE 4800
#define COMMA ","

    // EEPROM data
#define EEPROM_FILENUM_MSB              0
#define EEPROM_FILENUM_LSB              1
#define EEPROM_BRIGHTNESS               2
#define EEPROM_CONTRAST                 3

#define MENU_MAX_ITEMS                  4

    /*
     * PINS in use.
     */
    // Joystick (not used)
#define UP                              A1
#define RIGHT                           A2
#define DOWN                            A3
#define CLICK                           A4
//#define CAN_INT                       2
// #define GPS_RX_LED                   4
// #define GPS_TX_LED                   5
#define WIFLY_CTS                       A2 // Arduino CTS, WiFly RTS, input (from arduino), read this to see if it's ok to send data (HIGH indicates RX buffer full)
#define WIFLY_RTS                       A1 // Arduino RTS, WiFly CTS, output (from arduino), set to LOW to enable wifly to send data to arduino.  set to HIGH to disable.
#define WIFLY_TCP_CONNECTED             A3 // From WiFly. tells us when connected to AP.
#define WIFLY_WEB_CONNECT               A0 // To WiFly to tell it to open connection to remote host stored in config
#define WIFLY_WEB_CONNECTED             A4 // From WiFly. tells us when web connection has been opened.  ok to send data.
bool lastWiflyTcpConnected = false;
bool wiflyWebConnect = false;
bool lastWiflyWebConnect = false;
bool lastWiflyWebConnected = false;

#define BMS_BUZZER_INPUT                6

//#define LCDSERIAL_TX                    18/wiflySerial_TX // green wire
//#define LCDSERIAL_RX                    19/wiflySerial_RX // yellow wire

//#define GPSSERIAL_RX                    4, but shifted to Serial2.
//#define GPSSERIAL_TX                    5, but shifted to Serial2.

#define BATTERY_CURRENT_SENSOR_PIN      A12
#ifdef MOTOR_THERMISTOR
    #define MOTOR_THERMISTOR_PIN        A11
#endif //MOTOR_THERMISTOR
#define SD_CHIP_SELECT                  9 
// These are defined in the mcp library defaults.h"
// #define CANSHEILD_LED2               7
// #define CANSHEILD_LED3               8
// #define CAN_CHIP_SELECT              10
// #define SPI_MOSI                     11
// #define SPI_MISO                     12
// #define SPI_SCK                      13
//

#define lcdSerial Serial1
#define gpsSerial Serial3
#define wiflySerial Serial2

SdFat sd;
SdFile logFile;
SdFile nmeaFile;
SdFile readFile;
SdFile uploadFile;

//Print *stream = &Serial;
Print *stream = &logFile;
bool logFiles_open = false;
bool should_log = false;
bool log_ok = false;

uint8_t lcd_clear_count = 20;

CrystalFontz635 crystalFontz635;
Packet *packet;

CanOpenLite canOpenLite = CanOpenLite();
unsigned short failed_cs;
unsigned short failed_cs_last;
unsigned short failed_cs_diff;

    /*
     * stuff for time keeping
     */
int year;
uint8_t month, day, hour, minute, second;
unsigned long cur_gps_time;
unsigned long last_gps_time;
unsigned long diff_gps_time = 0;

unsigned long currentMillis = 0;
unsigned long lastSaveMillis = 0;
unsigned long lastDisplayMillis = 0;
unsigned long lastLogMillis = 0;
float millisSinceLastLog = 0;

/*
 * Vars needed by tinyGPS
 */
TinyGPS gps;
long lat, lon;
unsigned long fix_age, course;
long altitude;
float flat, flon, speed_GPS, fcourse = 0;
float prev_flat, prev_flon = 0;
float distance_GPS;
float distance_RPM;
float speed_RPM;
//float lastDistance_RPM;
float tripDistance_GPS = 0.0;
float tripDistance_RPM = 0.0;

float batteryWh = 0;
float batteryWhTotal = 0;
float milesPerKwh_GPS;
float whPerMile_GPS;
float milesPerKwh_RPM = 0;
float whPerMile_RPM = 0;
float milesPerKwh_Trip;
float whPerMile_Trip;

int motorThermistorReading = 0;
float c = 0;
#ifdef MOTOR_THERMISTOR
    #define motor5VActual 5.0
    float vOut;
    #define Z1 1000.0
    float z2;
#endif

long  batteryCurrentReadingTotal = 0;
float batteryCurrentReadingAvg;
float batteryCurrentReadingSingle;
float batteryCurrentAvg;
float batteryCurrentSingle;

float batteryW = 0;

uint16_t file_num = 0;

uint8_t buf_index;
char *buf_ptr;
char buffer[MAX_BUFSIZE+1];
PString bufferString((char*)buffer, sizeof(buffer));
char str00[] PROGMEM = "TangoLogger";
char str01[] PROGMEM = "Free RAM: ";
char str02[] PROGMEM = "CAN Init...";
char str03[] PROGMEM = "OK";
char str04[] PROGMEM = "FAIL";
char str05[] PROGMEM = ",";
char str06[] PROGMEM = "S:";
char str07[] PROGMEM = "C:";
char str08[] PROGMEM = "M:";
char str09[] PROGMEM = "mK:";
char str10[] PROGMEM = "D:";
char str11[] PROGMEM = "NO CANBUS";
char str12[] PROGMEM = "FULL";
char str13[] PROGMEM = "Wh:";
char str14[] PROGMEM = "Syncing files";
char str15[] PROGMEM = "DONE";
char str16[] PROGMEM = "%:";
char str17[] PROGMEM = "Brightness";
char str18[] PROGMEM = "G:";
char str19[] PROGMEM = "LogFile: ";
char str20[] PROGMEM = "B:";
char str21[] PROGMEM = "00000-LG.CSV";
char str22[] PROGMEM = "Does not exist";
char str23[] PROGMEM = "R:";
char str24[] PROGMEM = "!";
char str25[] PROGMEM = "I:";
char str26[] PROGMEM = "Init OK!";
char str27[] PROGMEM = "#LOGFMT 7 ";
char str28[] PROGMEM = " ";
char str29[] PROGMEM = { 0x10, 0x0 }; // arrow (for menu)
char str30[] PROGMEM = "Contrast";
char str31[] PROGMEM = "Start Upload...";
char str32[] PROGMEM = "Exit";
char str33[] PROGMEM = "WiFly Direct mode.";
char str34[] PROGMEM = "Use Serial Console.";
char str35[] PROGMEM = "Cancel (X) Btn exits";
PROGMEM char *strings[] = { str00, str01, str02, str03, str04, str05, str06, str07, str08, str09, 
                                    str10, str11, str12, str13, str14, str15, str16, str17, str18, str19,  
                                    str20, str21, str22, str23, str24, str25, str26, str27, str28, str29,
                                    str30, str31, str32, str33, str34, str35 };

bool gotGpsData = false;
uint32_t loopsSinceLastLog = 0;
SdoMessage sdoMsg;

typedef struct {
    uint16_t index;
    uint8_t subIndex;
    float scalingFactor;
    float value;
    unsigned long timeRequested;
    unsigned long timeReceived;
    char *description;
} ObjectDictionaryEntry;

ObjectDictionaryEntry throttleValueOD =         { 0x2620, 0x0, 0.0000305185094759972,   0, 0, 0, "tv" };
ObjectDictionaryEntry batteryVoltageOD =        { 0x5100, 0x1, 0.0625,                  0, 0, 0, "bv" };
ObjectDictionaryEntry batteryCurrentOD =        { 0x5100, 0x2, 0.0625,                  0, 0, 0, "bc" };
ObjectDictionaryEntry motorVoltageOD =          { 0x4600, 0xD, 0.0625,                  0, 0, 0, "mv" };
ObjectDictionaryEntry motorCurrentOD =          { 0x4600, 0xC, 1,                       0, 0, 0, "mc" };
ObjectDictionaryEntry heatsinkTempOD =          { 0x5100, 0x4, 1,                       0, 0, 0, "ht" };
ObjectDictionaryEntry motorTempControllerOD =   { 0x4600, 0x3, 1,                       0, 0, 0, "mt" };
ObjectDictionaryEntry speedOD =                 { 0x2721, 0x0, 0.0625,                  0, 0, 0, "sp" };
ObjectDictionaryEntry rpmOD =                   { 0x606c, 0x0, 1,                       0, 0, 0, "rpm" };
ObjectDictionaryEntry bdiOD =                   { 0x2790, 0x1, 1,                       0, 0, 0, "bdi" };

ObjectDictionaryEntry *idsToFetch[] = { &throttleValueOD, &batteryVoltageOD, &batteryCurrentOD, &motorVoltageOD, &motorCurrentOD, &heatsinkTempOD, &motorTempControllerOD, &speedOD, &rpmOD, NULL };

tCAN request;
tCAN response;

typedef enum {
    PROGRAMSTATE_NORMAL,
    PROGRAMSTATE_MENU,
    PROGRAMSTATE_UPLOAD,
    PROGRAMSTATE_WIFLYDIRECT,
    PROGRAMSTATE_DIALOG
} ProgramState;

ProgramState programState = PROGRAMSTATE_NORMAL;
bool stateChanged = true;
bool dataChanged = true;
bool displayParamsChanged = true;
uint8_t normalDisplayPage = 0;
uint8_t brightness;             // from 0-5 (display valid range 0-100)
uint8_t contrast;               // from 1-5 (display valid range 0-255, but docs say: "0-65 = very light, 66 = light, 95 = about right, 125 = dark, 126-255 = very dark
uint8_t currentMenuItem = 0;
byte byteRead;

bool bmsTripped = false;
bool lastBmsTripped = false;
unsigned long lastBmsTrippedTime;

SdBaseFile rootFile;
void setup() {
    Serial.begin(115200);
    lcdSerial.begin(115200);
    wiflySerial.begin ( 115200 );
    //wiflySerial.begin ( 230400 );
    //wiflySerial.begin ( 460800 );
    //wiflySerial.begin ( 500000 );
    crystalFontz635.init ( &lcdSerial );
    gpsSerial.begin(GPSRATE);
    analogReference(DEFAULT); // not sure this is necessary anymore...

    updateDisplayWithNewParams(); // Ensure brightness and contrast are setup correctly upon start.
    printString_P ( Serial, 0 );
    updateDisplay_Init();
    printlnString_P ( Serial, 26 );  // Init OK!
    rpmOD.value = 0; // set rpmOD to 0 since it's used to tell whether or not we should start logging

    pinMode ( WIFLY_RTS, OUTPUT );
    pinMode ( WIFLY_CTS, INPUT );
    pinMode ( BMS_BUZZER_INPUT, INPUT_PULLUP );
    digitalWrite ( WIFLY_RTS, LOW );
    pinMode ( WIFLY_TCP_CONNECTED, INPUT );
    pinMode ( WIFLY_WEB_CONNECTED, INPUT );
    pinMode ( WIFLY_WEB_CONNECT, OUTPUT );
    Serial.println( "Setting WIFLY_WEB_CONNECT to LOW SETUP" );
    wiflyWebConnect = LOW;
    setWiflyWebConnect ( LOW );
    rootFile.openRoot ( sd.vol() );
}

/*
 * in loop do these things:
 *  * perform all readings that should happen as often as possible u
 *     * (reading/averaging battery current from hall sensor)
 *     * getting GPS data
 *     * send and receive sevcon CAN msgs.
 *  * if 1 sec has passed:
 *     * perform further calcs that should only be done once a sec
 *     * write data to LCD
 *     * write data to log.
 */
void loop() {
    processUserInput();
    if ( displayParamsChanged ) {
        updateDisplayWithNewParams();
    }

    if ( ( programState != PROGRAMSTATE_UPLOAD ) && ( programState != PROGRAMSTATE_WIFLYDIRECT ) ) {
        gatherAndLogData();
    }

    if ( programState == PROGRAMSTATE_UPLOAD ) {
        loop_Upload();

    } else if ( programState == PROGRAMSTATE_WIFLYDIRECT ) {
        loop_WiflyDirect();

    }

    updateDisplay();
    stateChanged = false;
    dataChanged = false;
    displayParamsChanged = false;
}

typedef enum {
    UPLOADSTATE_BEGIN,
    UPLOADSTATE_FINDFILES,
    UPLOADSTATE_OPENINGFILE,
    UPLOADSTATE_SENDINGDATA,
    UPLOADSTATE_WAITING_FOR_SERVER_RESPONSE,
} UploadState;

UploadState uploadState = UPLOADSTATE_BEGIN;
dir_t dir;
uint16_t filesToUploadCount = 0;
uint16_t currentFileNumber = 0;

bool wiflyIsConnectionOpen = false;
bool wiflyInCommandMode = false;
char *stringToLookFor = NULL;
unsigned long start;
unsigned long startWaitingForResponseTime;

bool issueWiFlyCommand ( char *cmd, char *expectedResponse ) {
    bool found = false;
    strrev ( expectedResponse );
    writeWiFlySerial ( cmd );
    memset ( buffer, 0, MAX_BUFSIZE );
    start = millis();

    while ( ( millis() - start ) < 3000 ) {
        while ( wiflySerial.available() ) {
            byteRead = wiflySerial.read();
            Serial.write ( byteRead );
            memmove ( buffer + 1, buffer, MAX_BUFSIZE - 1 );
            buffer[MAX_BUFSIZE-1] = NULL;
            if ( byteRead == '\r' || byteRead == '\n' ) {
                buffer[0] = '|';
            } else {
                buffer[0] = byteRead;
            }
            if ( ! strncmp ( buffer, expectedResponse, strlen ( expectedResponse ) ) ) {
                strrev ( expectedResponse );
                return true;
            }
        }
    }
    return false;
}

void convertUploadFilename ( char *name, char *buf, bool eightDotThree) {
    uint8_t index = 8;
    memcpy ( buffer, name, 11 );
    if ( eightDotThree ) {
        buffer[index++] = '.';
    }
    buffer[index++] = 'U';
    buffer[index++] = 'P';
    buffer[index++] = 'L';
    buffer[index++] = NULL;
}

bool shouldUploadFile ( uint8_t name[] ) {
    //return ( ! ( strncmp ( (char *)name, "00021-NMGPS", 11 ) ) );
    if ( ( ! strncmp ( (char *)&name[5], "-NMGPS", 6 ) ) ||
         ( ! strncmp ( (char *)&name[5], "-LGCSV", 6 ) ) ) {
        convertUploadFilename ( (char *)name, buffer, true );
        return ! rootFile.exists ( buffer );
    }
    return false;
}

uint32_t bytesRead = 0;
uint32_t totalBytesRead = 0;
unsigned long fileStartTime;

bool getNextFileToUpload ( SdBaseFile *directory ) {
    while ( directory->readDir ( &dir ) == sizeof ( dir ) ) {
        Serial.print ( "name: " );
        Serial.print ( (char *)dir.name );
        Serial.print ( ", size: " );
        Serial.print ( dir.fileSize, DEC );
        Serial.print ( ", should upload? " );
        if ( shouldUploadFile ( dir.name ) ) {
            Serial.println ( "yes" );
            return true;
        }  else {
            Serial.println ( "no" );
        }
    }
    return false;
}

void deleteAllUploadFiles() {
    sd.vwd()->rewind();
    while ( sd.vwd()->readDir ( &dir ) == sizeof ( dir ) ) {
        if ( ! strncmp ( (char *)&dir.name[8], "UPL", 3 ) ) {
            SdFile::dirName(dir, buffer);
            Serial.print ( "Removing file: " );
            Serial.println ( buffer );
            SdBaseFile::remove ( &rootFile, (char *)buffer );
        }
    }
}

void loop_Upload() {
    // close file (if open)
    // get list of files
    // 
    // get file info
    // write $$$
    // write open
    // write data
    // write $$$ ???
    // write "close" ???
    uint32_t length;
    bool gotIt = false;
    bool open_success = false;
    int8_t readDirResponse;

    if ( stateChanged )  {
        uploadState = UPLOADSTATE_BEGIN;
        stringToLookFor = "*NEPO*";
        filesToUploadCount = 0;
        currentFileNumber = 0;
        uploadState = UPLOADSTATE_FINDFILES;
        sd.vwd()->rewind();
        while ( getNextFileToUpload ( sd.vwd() ) ) {
            filesToUploadCount++;
        }
        Serial.print ( "Files to upload: " );
        Serial.println ( filesToUploadCount, DEC );
        nmeaFile.close();
        logFile.close();
        should_log = false;

        digitalWrite ( WIFLY_RTS, HIGH );
        delayMicroseconds ( 1 );
        digitalWrite ( WIFLY_RTS, LOW );
        Serial.println ( "Toggling RTS" );
    }

    if ( uploadState == UPLOADSTATE_FINDFILES ) {
        if ( filesToUploadCount > currentFileNumber ) {
            uploadState = UPLOADSTATE_OPENINGFILE;
            sd.vwd()->rewind();
            getNextFileToUpload ( sd.vwd() );
        } else {
            programState = PROGRAMSTATE_NORMAL;
            stateChanged = true;
        }
    }

    if ( uploadState == UPLOADSTATE_OPENINGFILE ) {
        SdFile::dirName(dir, buffer);
        readFile.open ( buffer, O_READ );
        Serial.print ( "Processing File: " );
        Serial.println ( buffer );
        length = readFile.fileSize();
        Serial.print ( "Length: " );
        Serial.println ( length );
        year = ( ( dir.lastWriteDate >> 9 ) & 0x7F ) + 1980;
        month = ( dir.lastWriteDate >> 5 ) & 0x0F;
        day = dir.lastWriteDate & 0x1F;
        hour = ( dir.lastWriteTime >> 11 ) & 0x1F;
        minute = ( dir.lastWriteTime >> 5 ) & 0x3F;
        second = dir.lastWriteTime & 0x1F;

        Serial.println( "Setting WIFLY_WEB_CONNECT to HIGH 1" );
        setWiflyWebConnect ( HIGH );
        fileStartTime = millis();
        Serial.println ( "About to wait for WEB_CONNECTED" );
        while ( ! digitalRead ( WIFLY_WEB_CONNECTED ) ) {

        }
        Serial.println ( "Done waiting for WEB_CONNECTED" );

            // TODO: sprintf uses 2500 bytes. probably should be less lazy and construct the string by hand.
        sprintf ( buffer, "%04d_%02d_%02d-%02d_%02d_%02d-%s", year, month, day, hour, minute, second, ( ( dir.name[6] == 'L' ) ? "log.csv" : "gps.nmea" ) );

        writeWiFlySerial ( "POST /~altitude/TangoLogger/upload.cgi/" );
        writeWiFlySerial ( buffer );
        Serial.println ( buffer );
        writeWiFlySerial ( " HTTP/1.0\r\n" );
        writeWiFlySerial ( "Content-Length: " );
        bufferString.begin();
        bufferString.print ( length, DEC );
        writeWiFlySerial ( buffer );
        writeWiFlySerial ( "\r\n\r\n" );
        bytesRead = 0;
        totalBytesRead = 0;
        uploadState = UPLOADSTATE_SENDINGDATA;
        currentFileNumber++;
    }

    if ( uploadState == UPLOADSTATE_SENDINGDATA ) {
        //Serial.println ( "sending data" );
        unsigned long t1 = millis();
        while ( ( millis() - t1 < 100 ) && 
                ( bytesRead = readFile.read ( buffer, MAX_BUFSIZE ) ) > 0 ) {
            totalBytesRead += bytesRead;
            writeWiFlySerial ( buffer );
        } 
        
        if ( bytesRead <= 0 ) {
            readFile.close();
            Serial.println ( "Done with file" );
            uploadState = UPLOADSTATE_WAITING_FOR_SERVER_RESPONSE;
            startWaitingForResponseTime = millis();
            memset ( buffer, 0, MAX_BUFSIZE );
            Serial.println ( "data from server follows" );
        }
    }

    if ( uploadState == UPLOADSTATE_WAITING_FOR_SERVER_RESPONSE ) {
        while ( wiflySerial.available() ) {
            byteRead = wiflySerial.read();
            Serial.write ( byteRead );
            memmove ( buffer + 1, buffer, MAX_BUFSIZE - 1 );
            buffer[MAX_BUFSIZE-1] = NULL;
            buffer[0] = byteRead;
            if ( ! strncmp ( buffer, "*SOLC*", 6 ) ) {
                convertUploadFilename ( (char *)dir.name, buffer, true );
                open_success = uploadFile.open ( buffer, O_WRITE | O_CREAT );
                uploadFile.close();
                uploadState = UPLOADSTATE_FINDFILES;
                Serial.println( "Setting WIFLY_WEB_CONNECT to LOW 1" );
                setWiflyWebConnect ( LOW );
                Serial.print ( "creating 'done' file: " );
                Serial.print ( buffer );
                Serial.print ( ", success: " );
                Serial.println ( open_success );
                delay ( 500 );
                //success!
            } else if ( millis() - startWaitingForResponseTime > 5000 ) {
                Serial.println( "Setting WIFLY_WEB_CONNECT to LOW 2" );
                setWiflyWebConnect ( LOW );
                //fail.
            }
        }
    }
}

void updateDisplay_Upload() {
    if ( stateChanged ) {
        crystalFontz635.clearLCD();
        lcdPrintString ( 0, 0, "Uploading..." );
    }
    bufferString.begin();
    bufferString.print ( "File " );
    bufferString.print ( currentFileNumber, DEC );
    bufferString.print ( " of " );
    bufferString.print ( filesToUploadCount, DEC );
    lcdPrintString ( 1, 0, buffer );
    float percent = ((float)totalBytesRead / dir.fileSize) * 100.0;
    float tDiff = ( ( millis() - fileStartTime ) * ARDUINO_MILLIS_COMPENSATION_FACTOR ) / 1000;
    lcdPrintFloat ( 2, 0, tDiff, 6, 2 );
    lcdPrintFloat ( 2, 10, percent, 6, 2 );
    float bytesPerSecond = totalBytesRead / tDiff;
    lcdPrintFloat ( 3, 0, bytesPerSecond, 8, 2 );
}

bool writeWiFlySerial ( char buf[] ) {
    char *p;
    for ( p = buf; *p != (char)NULL; p++ ) {
        writeWiFlySerialByte ( (byte)*p );
    }
}

bool writeWiFlySerialInt ( int val, uint8_t size ) {
    int i;
    for ( i = size - 1; i >= 0; i-- ) {
        Serial.println ( val >> ( i * 8 )  & 0xFF );

        writeWiFlySerialByte ( val >> ( i * 8 )  & 0xFF );
    }
}

#define WIFLY_WRITE_TIMEOUT 3000
bool writeWiFlySerialByte ( byte b ) {
    int iterations = 0;
    unsigned long start = millis();
    while ( ( millis() - start ) < WIFLY_WRITE_TIMEOUT ) {
        iterations++;
        if ( LOW == digitalRead ( WIFLY_CTS ) ) {
            //Serial.write ( b );
            wiflySerial.write ( b );
            if ( iterations > 1 ) {
                Serial.print ( "iterations: " );
                Serial.println ( iterations, DEC );
            }
            return true;
        }
    }
    return false;
}

void updateDisplayWithNewParams() {
    brightness = ( EEPROM.read ( EEPROM_BRIGHTNESS ) > 5 ) ? 5 : EEPROM.read ( EEPROM_BRIGHTNESS );
    contrast = ( EEPROM.read ( EEPROM_CONTRAST ) > 5 ) ? 3 : EEPROM.read ( EEPROM_CONTRAST );
    if ( contrast < 1 ) {
        contrast = 3;
    }
    crystalFontz635.setBrightness ( brightness * 20, brightness * 20 );
    crystalFontz635.setContrast ( contrast * 16 + 48 );
}

void updateDisplay() {
    if ( stateChanged ) {
        Serial.print ( "stateChanged in display.  State is: " );
        Serial.println ( programState );
    }
    if ( millis() < 2000 ) {
        //updateDisplay_Init();

    } else if ( ( programState == PROGRAMSTATE_NORMAL ) &&
         ( ( dataChanged ) || ( stateChanged ) || ( ( millis() - lastDisplayMillis ) * ARDUINO_MILLIS_COMPENSATION_FACTOR > 1000 ) ) ) {
        lastDisplayMillis = millis();
        updateDisplay_Normal();

    } else if ( programState == PROGRAMSTATE_MENU ) {
        updateDisplay_Menu();
        
    } else if ( programState == PROGRAMSTATE_WIFLYDIRECT ) {
        updateDisplay_WiflyDirect();

    } else if ( ( programState == PROGRAMSTATE_UPLOAD ) &&
                ( ( stateChanged ) || ( ( millis() - lastDisplayMillis ) * ARDUINO_MILLIS_COMPENSATION_FACTOR > 1000 ) ) ) {
        lastDisplayMillis = millis();
        updateDisplay_Upload();
    } else if ( ( programState == PROGRAMSTATE_DIALOG ) && ( stateChanged ) ) {
        updateDisplay_Dialog();

    }

    if ( digitalRead ( WIFLY_TCP_CONNECTED ) != lastWiflyTcpConnected ) {
        lastWiflyTcpConnected = !lastWiflyTcpConnected;
        setLedBooleanGreen ( 0, lastWiflyTcpConnected );
    }

    if ( wiflyWebConnect != lastWiflyWebConnect ) {
        lastWiflyWebConnect = wiflyWebConnect;
        setLedBooleanYellow ( 1, wiflyWebConnect );
    }

    if ( digitalRead ( WIFLY_WEB_CONNECTED ) != lastWiflyWebConnected ) {
        lastWiflyWebConnected = !lastWiflyWebConnected;
        setLedBooleanRed ( 2, lastWiflyWebConnected );
    }

    if ( bmsTripped ) {
        setLedBooleanRed ( 3, true );
    } else {
        setLedBooleanRed ( 3, false );
    }

}

void setLedBooleanGreen ( uint8_t led, bool value ) {
    crystalFontz635.setLED ( led, 0, ( value ) ? 100 : 0 );
}

void setLedBooleanYellow ( uint8_t led, bool value ) {
    crystalFontz635.setLED ( led, ( value ) ? 100 : 0, ( value ) ? 100 : 0 );
}

void setLedBooleanRed ( uint8_t led, bool value ) {
    crystalFontz635.setLED ( led, ( value ) ? 100 : 0, 0 );
}

char *dialogStr0;

void updateDisplay_Dialog() {
    crystalFontz635.clearLCD();
    lcdPrintString ( 0, 0, dialogStr0 );
    lcdPrintString ( 3, 0, "Press OK or CANCEL" );
}

void processUserInput_Dialog ( Packet *packet ) {
    if ( ( CFA635_KEY_EXIT_PRESS == packet->data[0] ) || ( CFA635_KEY_ENTER_PRESS == packet->data[0] ) ) {
        programState = PROGRAMSTATE_NORMAL;
        stateChanged = true;
    }
}

void updateDisplay_Init() {
    crystalFontz635.clearLCD();
    lcdPrintString_P ( 0, 0, 0 ); // TangoLogger Init
    lcdPrintString_P ( 1, 0, 1 ); // Free Ram
    lcdPrintInt ( 1, 10, FreeRam(), 0, DEC );
    lcdPrintString_P ( 2, 0, 2 ); // CAN INIt
    #ifdef __AVR_ATmega2560__
    lcdPrintString ( 3, 0, "Mega" );
    #else
    lcdPrintString ( 3, 0, "Not Mega" );
    #endif

    Serial.println ( "About to init CAN" );
    if ( mcp2515_init(CANSPEED_1000) ) {
        lcdPrintString_P ( 2, 11, 3 ); // OK
    } else {
        lcdPrintString_P ( 2, 11, 4 ); // FAIL
    } 
    init_logger();
    lcdPrintString_P ( 3, 0, 19 );
    lcdPrintInt ( 3, 9, file_num, 0, DEC );

}

void updateDisplay_WiflyDirect() {
    if ( stateChanged ) {
        crystalFontz635.clearLCD();
        lcdPrintString_P ( 0, 0, 33 ); // WiFly Direct Mode
        lcdPrintString_P ( 1, 0, 34 ); // Use Serial Console
        lcdPrintString_P ( 2, 0, 35 ); // Cancel Btn exits
    }
}

/*
 *             1
 *   01234567890123456789
 *  +--------------------+
 * 0| Brightness       3 |
 * 1|>Contrast         4 |
 * 2| Start Upload...    |
 * 3| Exit               |
 *  +--------------------+
 */
void updateDisplay_Menu() {
    int i;
    if ( stateChanged ) {
        crystalFontz635.clearLCD();
        lcdPrintString_P ( 0, 1, 17 ); // Brightness
        lcdPrintString_P ( 1, 1, 30 ); // Contrast
        lcdPrintString_P ( 2, 1, 31 ); // Start Upload
        lcdPrintString_P ( 3, 1, 32 ); // Exit
    }

    if ( dataChanged || stateChanged || displayParamsChanged ) {
            // print the arrow for the current menu item.
        for ( i = 0; i < MENU_MAX_ITEMS; i++ ) {
            lcdPrintString_P ( i, 0, ( currentMenuItem == i ) ? 29 : 28 );
        }
        lcdPrintInt ( 0, 18, brightness, 1, DEC );
        lcdPrintInt ( 1, 18, contrast, 1, DEC );
    }
}


void processUserInput() {
    crystalFontz635.processInput();
        // Although this is in a while loop, we're mainly assuming only one keypress 
        // (CFA635_PACKET_TYPE_KEY_ACTIVITY)will happen during one invocation of this loop.  
        // I'm not sure if this is a good assumption or not...
    while ( packet = crystalFontz635.getNextPacket ( packet ) ) {
        if ( packet->type == CFA635_PACKET_TYPE_KEY_ACTIVITY ) {
            if ( programState == PROGRAMSTATE_NORMAL ) {
                processUserInput_Normal ( packet );

            } else if ( programState == PROGRAMSTATE_MENU ) {
                processUserInput_Menu ( packet );

            } else if ( programState == PROGRAMSTATE_WIFLYDIRECT ) {
                processUserInput_WiflyDirect ( packet );

            } else if ( programState == PROGRAMSTATE_UPLOAD ) {
                processUserInput_Upload ( packet );

            } else if ( programState == PROGRAMSTATE_DIALOG ) {
                processUserInput_Dialog ( packet );
            }
        }
    }
}
//            } else if ( CFA635_KEY_DOWN_PRESS == packet->data[0] ) {
//            } else if ( CFA635_KEY_RIGHT_PRESS == packet->data[0] ) {
//            } else if ( CFA635_KEY_LEFT_PRESS == packet->data[0] ) {
//            } else if ( CFA635_KEY_ENTER_PRESS == packet->data[0] ) {
//            } else if ( CFA635_KEY_EXIT_PRESS == packet->data[0] ) {
//            } else {
//                //crystalFontz635.dumpPacket ( "Got a valid packet: ", (uint8_t *)packet );
//            }


void processUserInput_Normal ( Packet *packet ) {
    if ( CFA635_KEY_UP_PRESS == packet->data[0] ) {
        normalDisplayPage = !normalDisplayPage;
        dataChanged = true;

    } else if ( CFA635_KEY_DOWN_PRESS == packet->data[0] ) {
        should_log = true;
        dataChanged = true;

    } else if ( CFA635_KEY_ENTER_PRESS == packet->data[0] ) {
        dataChanged = true;
        programState = PROGRAMSTATE_MENU;
        stateChanged = true;
    }
}

void processUserInput_Menu ( Packet *packet ) {
    if ( CFA635_KEY_UP_PRESS == packet->data[0] ) {
        if ( currentMenuItem > 0 ) {
            currentMenuItem--;
            dataChanged = true;
        }
    } else if ( CFA635_KEY_DOWN_PRESS == packet->data[0] ) {
        if ( currentMenuItem < ( MENU_MAX_ITEMS - 1 ) ) {
            currentMenuItem++;
            dataChanged = true;
        }
    } else if ( CFA635_KEY_LEFT_PRESS == packet->data[0] ) {
        if ( currentMenuItem == 0 ) {
            if ( brightness > 0 ) {
                brightness--;
                EEPROM.write ( EEPROM_BRIGHTNESS, brightness );
                displayParamsChanged = true;
            }
        } else if ( currentMenuItem == 1 ) {
            if ( contrast > 1 ) {
                contrast--;
                EEPROM.write ( EEPROM_CONTRAST, contrast );
                displayParamsChanged = true;
            }
        } else if ( currentMenuItem == 2 ) {
            deleteAllUploadFiles();
            dialogStr0 = "Deleting UPL files";
            programState = PROGRAMSTATE_DIALOG;
            currentMenuItem = 0;
            stateChanged = true;

        } else if ( currentMenuItem == 3 ) {
            currentMenuItem = 0;
            programState = PROGRAMSTATE_WIFLYDIRECT;
            stateChanged = true;
        }
    } else if ( CFA635_KEY_RIGHT_PRESS == packet->data[0] ) {
        if ( currentMenuItem == 0 ) {
            if ( brightness < 5 ) {
                brightness++;
                EEPROM.write ( EEPROM_BRIGHTNESS, brightness );
                displayParamsChanged = true;
            }
        } else if ( currentMenuItem == 1 ) {
            if ( contrast < 5 ) {
                contrast++;
                EEPROM.write ( EEPROM_CONTRAST, contrast );
                displayParamsChanged = true;
            }
        }
    } else if ( ( CFA635_KEY_EXIT_PRESS == packet->data[0] ) ||
                ( CFA635_KEY_ENTER_PRESS == packet->data[0] && currentMenuItem == 3 ) ) {
        programState = PROGRAMSTATE_NORMAL;
        currentMenuItem = 0;
        stateChanged = true;
    } else if ( CFA635_KEY_ENTER_PRESS == packet->data[0] && currentMenuItem == 2 ) {
        Serial.println ( "Setting ProgramState to upload and stateChanged to true" );
        programState = PROGRAMSTATE_UPLOAD;
        currentMenuItem = 0;
        stateChanged = true;
    }
}

bool lastWiFlyCTS = true;
unsigned long lastFoo = 0;
void processUserInput_WiflyDirect ( Packet *packet ) {
    if ( CFA635_KEY_EXIT_PRESS == packet->data[0] ) {
        programState = PROGRAMSTATE_NORMAL;
        stateChanged = true;
    } else if ( CFA635_KEY_LEFT_PRESS == packet->data[0] ) {
        digitalWrite ( WIFLY_RTS, LOW );
        Serial.println ( "Setting RTS to LOW" );
    } else if ( CFA635_KEY_RIGHT_PRESS == packet->data[0] ) {
        digitalWrite ( WIFLY_RTS, HIGH );
        Serial.println ( "Setting RTS to HIGH" );
    } else if ( CFA635_KEY_DOWN_PRESS == packet->data[0] ) {
        setWiflyWebConnect ( LOW );
        Serial.println ( "Setting WIFLY_WEB_CONNECT to LOW 5" );
    } else if ( CFA635_KEY_UP_PRESS == packet->data[0] ) {
        setWiflyWebConnect ( HIGH );
        Serial.println ( "Setting WIFLY_WEB_CONNECT to HIGH 6" );
    }
}

void processUserInput_Upload ( Packet *packet ) {
    if ( CFA635_KEY_EXIT_PRESS == packet->data[0] ) {
        programState = PROGRAMSTATE_NORMAL;
        Serial.println( "Setting WIFLY_WEB_CONNECT to LOW 3" );
        setWiflyWebConnect ( LOW );
        stateChanged = true;

    }
}


void loop_WiflyDirect() {
    int wiflyCTS = digitalRead ( WIFLY_CTS );
    if ( lastWiFlyCTS != wiflyCTS ) {
        Serial.print ( "wiflyCTS changed to " );
        Serial.println ( wiflyCTS, DEC );
        lastWiFlyCTS = wiflyCTS;
    }
    if ( millis() - lastFoo > 1000 ) {
        bool wiflyCTS = digitalRead ( WIFLY_CTS );
        int tcpConnected = analogRead ( WIFLY_TCP_CONNECTED );
        int webConnected = analogRead ( WIFLY_WEB_CONNECTED );
        /*
        Serial.print ( "Status: wiflyCTS: " );
        Serial.print ( wiflyCTS );
        Serial.print ( ", tcpConnected: " );
        Serial.print ( tcpConnected, DEC );
        Serial.print ( ", webConnected: " );
        Serial.println ( webConnected, DEC );
        */
        lastFoo = millis();
    }
    if ( stateChanged ) {
        Serial.println ( "Wifly Direct mode.  Don't forget line endinges (no line ending for '$$$', CRLF for everything else)" );
            // make sure no residual data in the serial buffer sent to the wifly.
        while ( Serial.available() ) {
            Serial.read();
        }
    }
    while ( wiflySerial.available() ) {
        Serial.write ( wiflySerial.read() );
    }

    while ( Serial.available() ) {
        byteRead = Serial.read();
        writeWiFlySerialByte ( byteRead );
    }
}

void gatherAndLogData() {
    loopsSinceLastLog++;
    currentMillis = millis();
    char gpsByte;
    gotGpsData = false;

        // get GPS Data
    while ( gpsSerial.available() ) {
        gpsByte = gpsSerial.read();
            //Serial.print ( gpsByte );
            //Serial.flush();
        if ( should_log ) {
            nmeaFile.print ( gpsByte );
        }
        if ( gps.encode( gpsByte ) ) {
            gps.f_get_position(&flat, &flon, &fix_age);
            speed_GPS = gps.f_speed_mph();
            if ( speed_GPS > 150 ) {
                speed_GPS = 0;
            }
            gps.crack_datetime( &year, &month, &day, &hour, &minute, &second, NULL, &fix_age );
            fcourse = gps.f_course();
            altitude = gps.altitude();
            gps.get_datetime ( NULL, &cur_gps_time, NULL );
            if ( cur_gps_time != last_gps_time ) {
                diff_gps_time = ( cur_gps_time - last_gps_time ) * 10;
                    // if time jumps drastically or goes backwards, set tDiff to 0, and reset the last time to the cur time
                    // (since we hope the next seconds worth of data will be more sane)
                if ( ( diff_gps_time < 0 ) || ( diff_gps_time > 30000 ) ) {
                    diff_gps_time = 0;
                    last_gps_time = cur_gps_time;
                } else {
                    gotGpsData = true;
                }
            }
        }
    }

        // Send CANOpen SDO requests for data if we haven't received or sent requests for it in a while.
        // wait until the system has been up for at least 3 seconds before starting.
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

        // Read battery current from hall sensor, and average it since the readings are somewhat jumpy.
        // Keep in mind this requires us to not overflow batteryCurrentReadingTotal.  With an ATMega running at 16MHz
        // this should be fine.  But if we move to something faster, we may want to reevaluate this assumption.
    batteryCurrentReadingSingle = analogRead ( BATTERY_CURRENT_SENSOR_PIN ) - 3; // the -3 is because the zero and 1.1A offset is off.
    batteryCurrentReadingTotal += batteryCurrentReadingSingle;
    batteryCurrentReadingAvg = batteryCurrentReadingTotal / loopsSinceLastLog;

        // if BMS_BUZZER_INPUT goes LOW, the BMS buzzer has tripped.  
    bmsTripped = ( digitalRead ( BMS_BUZZER_INPUT ) ) ? false : true;
    if ( bmsTripped && ! lastBmsTripped ) {
        lastBmsTrippedTime = millis();
    }

        // The millis() call on the current arduino board was using was WAY off, so i'm using a compensation factor to try to correct it.
        // From this point on, millisSinceLastLog should be closer to the proper time.
    currentMillis = millis();
    millisSinceLastLog = ( currentMillis - lastLogMillis ) * ARDUINO_MILLIS_COMPENSATION_FACTOR;
    if ( millisSinceLastLog >= 1000 ) {
        lastLogMillis = currentMillis;

        if ( ( gotGpsData ) && ( prev_flat != flat ) || ( prev_flon != flon ) ) {
            distance_GPS = gps.distance_between ( prev_flat, prev_flon, flat, flon ) * METERSTOMILES;
            if ( distance_GPS > 10 ) {
                distance_GPS = 0;
            }
            tripDistance_GPS += distance_GPS;
            prev_flat = flat;
            prev_flon = flon;
        } else  {
            distance_GPS = 0;
        }

        if ( rpmOD.value > 1200 || rpmOD.value < -500 ) {
            rpmOD.value = 0;
        }

        if ( ( should_log == false ) && ( rpmOD.value > 0 ) ) {
            should_log = true;
        }

        if ( ( should_log == true ) && ( logFiles_open == false ) ) {
            open_logFiles();
        }

        batteryCurrentAvg = convertBatteryCurrent ( batteryCurrentReadingAvg );
        batteryCurrentSingle = convertBatteryCurrent ( batteryCurrentReadingSingle );
        batteryW = batteryCurrentAvg * batteryVoltageOD.value;
        batteryWh = batteryW * millisSinceLastLog / MILLISPERHOUR;
        if ( batteryWh > 10 ) {
            batteryWh = 0;
        }
        batteryWhTotal += batteryWh;

        // Changed from Sevcon to Kelly KLS8080I.  Can't get the RPMs, so calculate the whPerMile_Trip stuff using GPS info for now.
        if ( tripDistance_GPS > 0.1) {
            whPerMile_Trip = batteryWhTotal / tripDistance_GPS;
            milesPerKwh_Trip = tripDistance_GPS / batteryWhTotal * 1000;
        }



            /*
             * The conversion of RPMs * diff_millis.  The calculated revs/mi is 789.0804025
             *
             * So distance_RPM = X rev   Y ms   1min    1sec     1 mile
             *                     --- *      * --    * ----- * ---------------
             *                     min          60 sec  1000ms   789.0804025 revs
             *
             * Reducing, distance_RPM (miles) = X rev/min * Y ms / 2.1121632997 * 10E-8.
             *
             *    speed_RPM = X rev    80.296 in    1 ft      1 mi      60 min
             *                ----- *  --------- * ------- * ------- *  ------ 
             *                  min     rev         12 in    5280 ft     1 hr
             * 
             * Reducing, speed_RPM (mph) = X rev/min * 0.076037879
             *
             *
             * Since that's too small a number for floating point on arduino to be precise, we use a
             * constant of 2.112632997 and then divide by 1E8 (100000000) later.
             */
        speed_RPM = rpmOD.value * 0.076037879;
        distance_RPM = rpmOD.value * millisSinceLastLog * 2.112632997; // remember to divide by 100000000 later!
        tripDistance_RPM += ( distance_RPM / 100000000 );
        if ( distance_GPS > 0 ) {
            whPerMile_GPS = batteryWh / distance_GPS;
            milesPerKwh_GPS = distance_GPS / batteryWh * 1000;
        } else {
            whPerMile_GPS = 0;
            milesPerKwh_GPS = 0;
        }
        if ( distance_RPM > 0 ) {
            whPerMile_RPM = batteryWh / ( distance_RPM / 100000000 );
            milesPerKwh_RPM = distance_RPM / 100000 / batteryWh;
            if ( whPerMile_RPM >= 250 ) {
                whPerMile_RPM = 250;
            }
            if ( milesPerKwh_RPM >= 99 ) {
                milesPerKwh_RPM = 99;
            }
            //whPerMile_Trip = batteryWhTotal / tripDistance_RPM;
            //milesPerKwh_Trip = tripDistance_RPM / batteryWhTotal * 1000;
        } else {
            whPerMile_RPM = 0;
            milesPerKwh_RPM = 0;
        }

#ifdef MOTOR_THERMISTOR
        motorThermistorReading = analogRead(MOTOR_THERMISTOR_PIN);
        vOut = motor5VActual / 1024.0 * (float)motorThermistorReading;
        z2 = ( -1 * vOut * Z1 ) / ( vOut - motor5VActual );
        c = -2 * pow(10,-5) * pow( z2, 2)  + 0.1638 * z2 - 120.28;
        //f = c * 9 / 5 + 32;
#endif // MOTOR_THERMISTOR  29840

        if ( should_log ) {
            printLong ( *stream, currentMillis, DEC );
            printFloat ( *stream, (float)currentMillis * ARDUINO_MILLIS_COMPENSATION_FACTOR, 4 );
            printInt ( *stream, diff_gps_time, DEC );
            printLong ( *stream, loopsSinceLastLog, DEC );
            printIntLeadingZero ( *stream, year ); printIntLeadingZero ( *stream, month ); printIntLeadingZero ( *stream, day ); printString_P ( *stream, 5 );
            printIntLeadingZero ( *stream, hour ); printIntLeadingZero ( *stream, minute ); printIntLeadingZero ( *stream, second ); printString_P ( *stream, 5 );
            printInt ( *stream, fix_age, DEC );
            printFloat ( *stream, speed_GPS, 2 );
            printFloat ( *stream, speed_RPM, 2 );
            printFloat ( *stream, speedOD.value, 2 );
            printFloat ( *stream, flat, 5 );
            printFloat ( *stream, flon, 5 );
            printFloat ( *stream, fcourse, 2 );
            printInt ( *stream, altitude, DEC );
            printInt ( *stream, failed_cs_diff, DEC );
            printFloat ( *stream, distance_GPS, 5 );
            printFloat ( *stream, distance_RPM / 100000000, 5 );
            printFloat ( *stream, rpmOD.value, 5 );
            printFloat ( *stream, batteryVoltageOD.value, 3 );
            printFloat ( *stream, batteryCurrentReadingTotal, DEC );
            printFloat ( *stream, batteryCurrentAvg, 5 );
            printFloat ( *stream, batteryCurrentReadingSingle, DEC );
            printFloat ( *stream, batteryCurrentSingle, 5 );
            printFloat ( *stream, batteryCurrentOD.value, 5 );
            printFloat ( *stream, batteryWh, 5 );
            printFloat ( *stream, batteryWhTotal, 5 );
            printFloat ( *stream, motorCurrentOD.value, 4 );
            printFloat ( *stream, motorVoltageOD.value, 4 );
            printFloat ( *stream, (motorCurrentOD.value * motorVoltageOD.value), 4 );
            printFloat ( *stream, whPerMile_GPS, 5 );
            printFloat ( *stream, whPerMile_RPM, 5 );
            printFloat ( *stream, whPerMile_Trip, 5 );
            printFloat ( *stream, milesPerKwh_GPS, 5 );
            printFloat ( *stream, milesPerKwh_RPM, 5 );
            printFloat ( *stream, milesPerKwh_Trip, 5 );
            printFloat ( *stream, motorTempControllerOD.value, 4 );
            printFloat ( *stream, c, 2 );
            printInt ( *stream, motorThermistorReading, DEC );
            printFloat ( *stream, throttleValueOD.value, 4 );
            printFloat ( *stream, bdiOD.value, 4 );
            printFloat ( *stream, heatsinkTempOD.value, 4 );
            printLine ( *stream );
        }

            // reset distance_GPS in case we do another round before getting another
            // set of GPS data, we don't re-calcualte wh/mi with a bogus distance_GPS.
        distance_GPS = 0;
        loopsSinceLastLog = 0;
        batteryCurrentReadingTotal = 0;
        last_gps_time = cur_gps_time;
    }


    if ( ( currentMillis - lastSaveMillis ) >= 5000 ) {
        gps.stats ( NULL, NULL, &failed_cs );
        failed_cs_diff = failed_cs - failed_cs_last;
        failed_cs_last = failed_cs;
        logFile.sync();
        nmeaFile.sync();
        lastSaveMillis = currentMillis;
    }
}


/*
 *             1
 *   01234567890123456789
 *  +--------------------+
 * 0|B:XXX.X X.XX I:XXX.X
 * 1|M:XXX.X I:XXX.X %XX!
 * 1|G:XX.X R:XX.X C:XX.X
 * 2|mK:XX.XX XX.XX C:XXX
 * 3|D:XX.X Wh:XXXX XX:XX     
 *  +--------------------+
 */
void updateDisplay_Normal() {
    if ( stateChanged || dataChanged || lcd_clear_count++ > 10 ) {
            // Print LCD labels once per 10 sec or if any major changes.
        crystalFontz635.clearLCD();
        if ( normalDisplayPage == 0 ) {
            lcdPrintString_P ( 0, 0, 20 ); // B:
            lcdPrintString_P ( 0, 13, 25 ); // I:
            
            lcdPrintString_P ( 1, 0, 18 );
            lcdPrintString_P ( 1, 7, 23 );
            lcdPrintString_P ( 1, 14, 7 );

            lcdPrintString_P ( 2, 0, 9 ); // mk:
            lcdPrintString_P ( 2, 15, 7 ); // C:

            lcdPrintString_P ( 3, 0, 10 ); // D:
            lcdPrintString_P ( 3, 7, 13 ); // D:
        } else {
            lcdPrintString_P ( 0, 0, 20 ); // B:
            lcdPrintString_P ( 0, 13, 25 ); // I:
            lcdPrintString_P ( 1, 0, 8 ); // M:
            lcdPrintString_P ( 1, 8, 25 ); // I:
            lcdPrintString_P ( 1, 16, 16 ); // %:
        }
        lcd_clear_count = 0;
        dataChanged = false;
    }

    lcdPrintFloat ( 0, 2, batteryVoltageOD.value, 5, 1 );
    lcdPrintFloat ( 0, 7, ( batteryVoltageOD.value / 36 ), 5, 2 );

    if ( batteryCurrentAvg <= -100 ) {
        batteryCurrentAvg = 0;
    }
    if ( batteryCurrentAvg < 0 ) {
        lcdPrintFloat ( 0, 15, batteryCurrentAvg, 4, 1 );
    } else {
        lcdPrintFloat ( 0, 15, batteryCurrentAvg, 5, 1 );
    }

    if ( normalDisplayPage == 0 ) {

        lcdPrintFloat ( 1, 2, speed_GPS, 4, 1 );
        lcdPrintFloat ( 1, 9, speed_RPM, 4, 1 );
        lcdPrintFloat ( 1, 16, speedOD.value, 4, 1 );
        if ( ! should_log ) {
            lcdPrintString_P ( 1, 19, 24 );
        }

        // Can't get RPM, so use GPS info for now.
        //lcdPrintFloat ( 2, 3, milesPerKwh_RPM, 5, 2 );
        lcdPrintFloat ( 2, 3, milesPerKwh_GPS, 5, 2 );
        lcdPrintFloat ( 2, 9, milesPerKwh_Trip, 5, 2 );
        //lcdPrintInt ( 2, 17, motorTempControllerOD.value, 3, DEC );
        lcdPrintInt ( 2, 17, c, 3, DEC );

        // Can't get RPM, so use GPS info for now.
        //lcdPrintFloat ( 3, 2, tripDistance_RPM, 4, 1 );
        lcdPrintFloat ( 3, 2, tripDistance_GPS, 4, 1 );
        lcdPrintInt ( 3, 10, int ( batteryWhTotal ), 4, DEC );
    } else {
        lcdPrintFloat ( 1, 2, motorVoltageOD.value, 5, 1 );
        lcdPrintFloat ( 1, 10, motorCurrentOD.value, 5, 1 );
        if ( bdiOD.value >= 100 ) {
            lcdPrintString_P ( 1, 16, 12 );
        } else {
            lcdPrintInt ( 1, 17, (long) bdiOD.value, 3, DEC );
        }
    }
    int tzHour = hour + TIMEZONEOFFSET;
    if ( tzHour < 0 ) {
        tzHour += 24;
    } else if ( tzHour >= 24 ) {
        tzHour -= 24;
    }
    bufferString.begin();
    printIntLeadingZero ( bufferString, tzHour );
    bufferString.print ( ":" );
    printIntLeadingZero ( bufferString, minute );
    lcdPrintString ( 3, 15, buffer );
}

void printIntLeadingZero ( Print &stream, int digits ) {
  if ( digits < 10 ) {
      stream.print ( '0' );
  }
  stream.print ( digits );
}

void printFloat ( Print &stream, float f, int places ) {
    stream.print ( f, places );
    stream.print ( COMMA );
}

void printInt ( Print &stream, int i, int type ) {
    stream.print ( i, type );
    stream.print ( COMMA );
}

void printLong ( Print &stream, long l, int type ) {
    stream.print ( l, type );
    stream.print ( COMMA );
}

void printString ( Print &stream, char *s ) {
    stream.print ( s );
    //stream.print ( COMMA );
}

void printLine ( Print &stream ) {
    stream.println();
}

void printString_P ( Print &stream, int index ) {
    strcpy_P ( buffer, (char*)pgm_read_word ( &(strings[index]) ) );
    stream.print ( buffer );
}

void printlnString_P ( Print &stream, int index ) {
    printString_P ( stream, index );
    stream.println();
}

void lcdPrintString_P ( uint8_t row, uint8_t col, int index ) {
    strcpy_P ( buffer, (char*)pgm_read_word ( &(strings[index]) ) );
    crystalFontz635.printAt ( row, col, buffer );
}

void lcdPrintString ( uint8_t row, uint8_t col, char *s ) {
    crystalFontz635.printAt ( row, col, s );
}

void lcdPrintFloat ( uint8_t row, uint8_t col, float f, uint8_t padding, uint8_t places ) {
    uint8_t count = 0;
    if ( int ( f / 1000 ) > 0 ) {
        count = 4;
    } else if ( int ( f / 100 ) > 0 ) {
        count = 3;
    } else if ( int ( f / 10 ) > 0 ) {
        count = 2;
    } else {
        count = 1;
    }
    count += places + 1;
    if ( f < 0 ) {
        count++;
    }
    bufferString.begin();
    for ( uint8_t i = padding; i > count; i-- ) {
        bufferString.print ( " " );
    }
    bufferString.print ( f, places );
    crystalFontz635.printAt ( row, col, buffer );
}

void lcdPrintInt ( uint8_t row, uint8_t col, long l, uint8_t padding, uint8_t type ) {
    uint8_t count = 0;
    if ( int ( l / 1000 ) > 0 ) {
        count = 4;
    } else if ( int ( l / 100 ) > 0 ) {
        count = 3;
    } else if ( int ( l / 10 ) > 0 ) {
        count = 2;
    } else {
        count = 1;
    }
    if ( l < 0 ) {
        count++;
    }
    bufferString.begin();
    for ( uint8_t i = padding; i > count; i-- ) {
        bufferString.print ( " " );
    }
    bufferString.print ( l, type );
    crystalFontz635.printAt ( row, col, buffer );
}

float convertBatteryCurrent ( float batteryCurrentReading ) {
    float batteryCurrent;
    // return batteryCurrentReading * -0.9638554 + 799.0361446 + 2.5; // For CSLA2DK Backwards
    //batteryCurrent = batteryCurrentReading * 0.9638554 - 799.0361446 + 2.5 - 8.3; // For CSLA2DK Forwards
    batteryCurrent = batteryCurrentReading * 1.1298 - 800; // For CSLA2DK Forwards

    // return batteryCurrentReading * -05421687 + 449.4578313 + 2.5; // For CSL1EJ Backwards
    // return batteryCurrentReading * 0.9638554 - 449.4578313 + 2.5; // For CSLA1EJ Forwards
    return ( batteryCurrent > 0 ) ? batteryCurrent : 0;
}

void tangoDateTimeCallback(uint16_t* date, uint16_t* time) {
    // return date using FAT_DATE macro to format fields
    *date = FAT_DATE ( year, month, day );

    // return time using FAT_TIME macro to format fields
    *time = FAT_TIME ( hour, minute, second );
}

/*
 * Filename and SD card utility functions.
 */
uint16_t getCurrentFileNum() {
    file_num = EEPROM.read ( EEPROM_FILENUM_MSB ) << 8 | EEPROM.read ( EEPROM_FILENUM_LSB );
    if ( file_num == 0xFFFF ) {
        file_num = 0;
    }
    return file_num;
}

void init_logger() {
    // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
    // breadboards.  use SPI_FULL_SPEED for better performance.
    // if SD chip select is not SS, the second argument to init is CS pin number
    if (! sd.begin(SD_CHIP_SELECT, SPI_FULL_SPEED)) {
        sd.initErrorHalt();
    }
    SdFile::dateTimeCallback(tangoDateTimeCallback);

    file_num = getCurrentFileNum();

    printString_P ( Serial, 19 ); //log file: 
    Serial.println ( file_num, DEC );
}

void create_filename ( uint16_t num ) {
    strcpy_P ( buffer, (char*)pgm_read_word ( &(strings[21]) ) ); // base filename
    buffer[0] = num / 10000 + '0';
    buffer[1] = num % 10000 / 1000 + '0';
    buffer[2] = num % 1000 / 100 + '0';
    buffer[3] = num % 100 / 10 + '0';
    buffer[4] = num % 10 + '0';
}

void open_logFiles() {
    file_num = getCurrentFileNum();
    create_filename ( file_num );
    if ( ! logFile.open ( buffer, O_WRITE | O_CREAT ) ) {
        log_ok = false;
    }

    buffer[6] = 'N';
    buffer[7] = 'M';
    buffer[9] = 'g';
    buffer[10] = 'p';
    buffer[11] = 's';
    if ( ! nmeaFile.open ( buffer, O_WRITE | O_CREAT ) ) {
        log_ok = false;
    }
    logFiles_open = true;
    EEPROM.write ( EEPROM_FILENUM_MSB, ( ( file_num + 1 ) >> 8 ) & 0xFF );
    EEPROM.write ( EEPROM_FILENUM_LSB, ( file_num + 1 ) & 0xFF );
    printString_P ( *stream, 27 ); // Output Format type
    printFloat ( *stream, ARDUINO_MILLIS_COMPENSATION_FACTOR, 6 ); // include ardunio millis compensation for later analysis.
    printLine ( *stream );
}

void setWiflyWebConnect ( boolean value ) {
    wiflyWebConnect = value;
    digitalWrite ( WIFLY_WEB_CONNECT, value );
}
