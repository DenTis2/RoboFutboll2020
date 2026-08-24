#include "MotorController.hpp"
#include <sstream>
#include <algorithm>

MotorController::MotorController(mySerial& bus_, const RobotConfig& cfg)
: bus(bus_), config(cfg) {}

void MotorController::setSpeed(float left, float right) {
    left = std::clamp(left, -(float)config.maxSpeed, (float)config.maxSpeed);
    right = std::clamp(right, -(float)config.maxSpeed, (float)config.maxSpeed);

    std::stringstream ss;
    ss <<"M:"<< (int)left << ";" << (int)right<<";"; 
    bus.SendLine(ss.str());
}
