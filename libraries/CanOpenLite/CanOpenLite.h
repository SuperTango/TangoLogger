#ifndef canopenlite__h
#define canopenlite__h

#include "Arduino.h"

typedef enum {
    SEGMENT_DOWNLOAD = 0,
    INITIATING_DOWNLOAD = 1,
    INITIATING_UPLOAD = 2,
    SEGMENT_UPLOAD = 3,
    ABORT_TRANSFER = 4
} SdoMessageType;

typedef struct {
    uint8_t type;
    uint8_t length;
    uint16_t index;
    uint8_t subIndex;
    uint32_t data;
} SdoMessage;

class CanOpenLite {
    public:
        CanOpenLite();
        void sdoMessageFromBuffer ( SdoMessage *sdoMessage, uint8_t buffer[] );
        void sdoMessageToBuffer ( SdoMessage *sdoMessage, uint8_t *buffer );
};
    



#endif //canopenlite__h
