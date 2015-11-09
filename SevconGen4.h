/**
 * Copyright (c) 2015 Alex Tang
 */
#ifndef sevcongen4__h
#define sevcongen4__h

#include "mcp2515.h"
#include "CanOpenLite.h"
#include "MotorController.h"

typedef struct {
    uint16_t index;
    uint8_t subIndex;
    float scalingFactor;
    float value;
    unsigned long timeRequested;
    unsigned long timeReceived;
    char *description;
} ObjectDictionaryEntry;

class SevconGen4 : public MotorController {
  public:
    SevconGen4();
    void init();
    bool processData();

  private:
    tCAN request;
    tCAN response;
    SdoMessage sdoMsg;
    CanOpenLite canOpenLite;
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
    ObjectDictionaryEntry *idsToFetch[10];

};
#endif //sevcongen4__h

