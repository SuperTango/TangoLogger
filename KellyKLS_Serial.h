/**
 * Copyright (c) 2015 Alex Tang
 */
#ifndef kellykls_serial__h
#define kellykls_serial__h

#define KLS8080I_LOGDATA_BUFSIZE 19
#define REQUEST_TYPE_3A 0x3A
#define REQUEST_TYPE_3B 0x3B

class KellyKLS_Serial {
  public:
    KellyKLS_Serial();
    void init ( Stream *controllerStream );
    bool readData();

    float throttlePercent;
    bool reverseSwitch;
    float batteryVoltage;
    float controllerTemp;
    uint16_t rpm;
    float motorCurrent;
    unsigned long last3APacketReceivedMillis;
    unsigned long last3BPacketReceivedMillis;

  private:
    Stream *controllerStream;
    bool validateChecksum();
    unsigned long lastControllerRequestTime;
    uint8_t requestType;
    uint8_t controllerBuffer[KLS8080I_LOGDATA_BUFSIZE+1];
    uint8_t controllerBufferIndex;
    uint8_t lastPrintedControllerBufferIndex;
};
#endif //kellykls_serial__h
