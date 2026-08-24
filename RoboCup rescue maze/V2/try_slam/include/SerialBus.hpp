#pragma once
#include <asio.hpp>
#include <string>
#include <mutex>

class SerialBus {
public:
    SerialBus(const std::string& portName, unsigned int baud);

    bool available();
    bool readByte(uint8_t& b);
    bool readBytes(uint8_t* buf, size_t n);
    bool readLine(std::string& line);       
    void writeMotor(const std::string& line); 
    void writeDebug(const std::string& line); 
    size_t readSome(uint8_t* data, size_t maxLen);
private:
    asio::io_service io;
    asio::serial_port port;
    asio::streambuf buf;
    std::mutex writeMutex;
};
