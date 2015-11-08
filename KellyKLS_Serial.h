/**
 * Copyright (c) 2015 Alex Tang
 */
#ifndef kellykls_serial__h
#define kellykls_serial__h

#define KLS8080I_RECEIVE_BUFSIZE 19
#define KLS8080I_SENDDATA_BUFSIZE 4
#define REQUEST_TYPE_3A 0x3A
#define REQUEST_TYPE_3B 0x3B

class KellyKLS_Serial {
  public:
    KellyKLS_Serial();
    void init ( Stream *controllerStream );
    bool processData();

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
    uint8_t receiveBufferIndex;
    unsigned long lastControllerRequestTime;
    uint8_t requestType;
    uint8_t sendBuffer[KLS8080I_RECEIVE_BUFSIZE];
    uint8_t receiveBuffer[KLS8080I_SENDDATA_BUFSIZE + 1];
    //void dumpReceiveBuffer();
};
#endif //kellykls_serial__h
