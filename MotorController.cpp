/**
 * Copyright (c) 2015 Alex Tang
 */
#if ARDUINO>=100
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif

#include "MotorController.h"

MotorController::MotorController() {
}

void MotorController::init() {
    throttlePercent = 0.0;
    batteryVoltage = 0.0;
    batteryCurrent = 0.0;
    motorVoltage = 0.0;
    motorCurrent = 0.0;
    controllerTemp = 0.0;
    motorTemp = 0.0;
    rpm = 0.0;
    direction = 1;
    bdi = 0.0;
    speed = 0.0;
    initSuccess = false;
}
