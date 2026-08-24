#pragma once
#include "mySerial.h"
#include "RobotConfig.hpp"

class MotorController {
public:
    MotorController(mySerial& bus, const RobotConfig& config);
    void setSpeed(float left, float right); // -maxSpeed .. maxSpeed

private:
    mySerial& bus;
    const RobotConfig& config;
};
