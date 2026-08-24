#pragma once

struct RobotConfig {
    double width;       // ширина робота в метрах
    double length;      // длина робота в метрах
    double wheelBase;   // расстояние между колёсами
    double maxSpeed;    // максимальная скорость моторов

    // PID для курса
    double pid_kp;
    double pid_ki;
    double pid_kd;

    double tankTurnThreshold; // угол ошибки для танковых разворотов (рад)
};
