#pragma once
#include <vector>
#include <cstdint>
#include "SerialBus.hpp"
#include <thread>
#include <mutex>
#include "mySerial.h"
#include <map>
struct LidarPoint {
    double angle;    
    double distance; 
};

class LidarReader {
public:
    explicit LidarReader(mySerial* serial,mySerial* pico);
    ~LidarReader();
    bool readScan(std::vector<LidarPoint>& scan);
    double getright();
    double getleft();

private:
    mySerial* serial;
    mySerial* pico;
    std::map<int, LidarPoint> latestScan; // угол -> точка
    std::map<int,int> time;
    std::mutex scanMutex;
    std::thread uartThread;
    std::atomic<bool> running;

    void uartReaderThread(); // поток чтения UART
    long long counter;
    double right,left;
};
