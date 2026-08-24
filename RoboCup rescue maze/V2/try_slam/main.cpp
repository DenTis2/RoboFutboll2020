// main.cpp
#include "SerialBus.hpp"
#include "LidarReader.hpp"
#include "MotorController.hpp"
#include "Map.hpp"
#include "SLAM.hpp"
#include "Explorer.hpp"
#include "RobotConfig.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>
#include <atomic>

static std::atomic<bool> stopFlag{false};
void sigint_handler(int) { stopFlag = true; }

int main(int argc, char** argv) {
    signal(SIGINT, sigint_handler);
    try {
        RobotConfig cfg;
        cfg.width = 0.25;      // m
        cfg.length = 0.30;     // m
        cfg.wheelBase = 0.15;  // m
        cfg.maxSpeed = 100.0;  // motor units (as in firmware)

        // PID (примерные)
        cfg.pid_kp = 2.0;
        cfg.pid_ki = 0.0;
        cfg.pid_kd = 0.5;
        cfg.tankTurnThreshold = 0.35; // rad ~20deg

        // Открой порт (укажи свой порт/baud если нужно)
        std::string port = "/dev/ttyUSB0";
        unsigned int baud = 115200;
        mySerial serial(port, baud);
        mySerial pico("/dev/ttyAMA0",baud);
        LidarReader lidar(&serial,&pico);
        MotorController motor(pico, cfg);
        Map map(500, 500, 0.05); // 25m x 25m при resolution=0.05
        map.inflateObstacles(cfg.width, cfg.length);
        SLAM slam(lidar, map, motor, cfg);
        Explorer explorer(map, slam, motor, cfg);

        serial.SendLine("Start exploration");

        // основной цикл
        const std::chrono::milliseconds loopDelay(10); // 100 Hz main loop cap
        while (!stopFlag) {
            slam.step();
            explorer.step();
            std::this_thread::sleep_for(loopDelay);
        }

        // остановим робота
        motor.setSpeed(0.0f, 0.0f);
        serial.SendLine("Stopping");
    } catch (const std::exception &ex) {
        std::cerr << "Fatal error: " << ex.what() << std::endl;
        return 2;
    } catch (...) {
        std::cerr << "Unknown fatal error\n";
        return 3;
    }
    std::cout << "Exited cleanly\n";
    return 0;
}
