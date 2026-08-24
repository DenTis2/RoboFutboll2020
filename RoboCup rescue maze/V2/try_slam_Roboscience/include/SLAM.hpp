// SLAM.hpp — обновлённый
#pragma once
#include "LidarReader.hpp"
#include "Map.hpp"
#include "MotorController.hpp"
#include "RobotConfig.hpp"
#include <Eigen/Dense>
#include <vector>
#include <chrono>

class SLAM {
public:
    SLAM(LidarReader& lidar, Map& map, MotorController& motor, const RobotConfig& cfg);

    // основной шаг (читать лидар, матчить, обновлять карту)
    void step();

    // вернуть текущую позу (x, y, yaw)
    Eigen::Vector3d getPose() const;

private:
    LidarReader& lidar;
    Map& map;
    MotorController& motor;
    const RobotConfig& config;

    Eigen::Vector3d pose;                 // x, y, yaw
    std::vector<LidarPoint> lastScan;     // предыдущий скан (raw points)

    // odometry internal pose (predicted by encoders) - useful to keep
    Eigen::Vector3d odomPose;

    // time
    std::chrono::steady_clock::time_point lastTime;

    // конвертация поляр->декарт
    Eigen::Vector2d polarToCartesian(double angle, double distance) const;

    // одометрия: интеграция по энкодерам
    void integrateOdometry(double dt);

    // основной матчинг сканов: возвращает true если удалось посчитать смещение
    bool scanMatch(const std::vector<LidarPoint>& newScan,
                   double &out_tx, double &out_ty, double &out_dtheta);

    // небольшой утилитарный: медиана
    static double median_of_vector(std::vector<double>& v);
};
