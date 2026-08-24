#pragma once
#include "Map.hpp"
#include "SLAM.hpp"
#include "MotorController.hpp"
#include "RobotConfig.hpp"
#include <Eigen/Dense>
#include <vector>

class Explorer {
public:
    Explorer(Map& map, SLAM& slam, MotorController& motor, const RobotConfig& cfg);
    void step();

private:
    Map& map;
    SLAM& slam;
    MotorController& motor;
    const RobotConfig& config;

    std::vector<Eigen::Vector2d> currentPath;

    Eigen::Vector2d findNextFrontier();
    std::vector<Eigen::Vector2d> planPath(const Eigen::Vector2d& start, const Eigen::Vector2d& goal);
    void moveAlongPath();

    // PID-переменные
    double last_error = 0.0;
    double integral = 0.0;
};
