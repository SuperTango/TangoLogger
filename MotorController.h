/**
 * Copyright (c) 2015 Alex Tang
 */
#ifndef motor_controller__h
#define motor_controller__h

class MotorController {
  public:
    MotorController();
    virtual bool processData() = 0;
    void init();

    bool initSuccess;
    float throttlePercent;
    float batteryVoltage;
    float batteryCurrent;
    float motorVoltage;
    float motorCurrent;
    float controllerTemp;
    float motorTemp;
    float rpm;
    float speed;
    int8_t direction;
    float bdi;
};
#endif //motor_controller__h
