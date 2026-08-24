#include "SerialBus.hpp"

SerialBus::SerialBus(const std::string& portName, unsigned int baud)
    : port(io, portName)
{
    port.set_option(asio::serial_port_base::baud_rate(baud));
}

bool SerialBus::readLine(std::string& line) {
    try {
        asio::read_until(port, buf, '\n');
        std::istream is(&buf);
        std::getline(is, line);
        return true;
    } catch (...) {
        return false;
    }
}

bool SerialBus::readByte(uint8_t& b) {
    try {
        size_t n = port.read_some(asio::buffer(&b, 1));
        return n == 1;
    } catch (...) { return false; }
}

size_t SerialBus::readSome(uint8_t* data, size_t maxLen) {
    try {
        return port.read_some(asio::buffer(data, maxLen));
    } catch (...) {
        return 0;
    }
}

bool SerialBus::available() {
      return 1;
//    return port.available(); // или size_t, если есть возвращаемое число байт
}

bool SerialBus::readBytes(uint8_t* buf, size_t n) {
    try {
        size_t total = 0;
        while(total < n) {
            size_t got = port.read_some(asio::buffer(buf + total, n - total));
            if(got == 0) return false;
            total += got;
        }
        return true;
    } catch (...) { return false; }
}

/*
bool SerialBus::readByte(uint8_t& b) {
    try {
        asio::read(port, asio::buffer(&b, 1));
        return true;
    } catch (...) {
        return false;
    }
}

bool SerialBus::readBytes(uint8_t* buf, size_t n) {
    try {
        asio::read(port, asio::buffer(buf, n));
        return true;
    } catch (...) {
        return false;
    }
}
*/

void SerialBus::writeMotor(const std::string& line) {
    std::lock_guard<std::mutex> lock(writeMutex);
    asio::write(port, asio::buffer("M;" + line + "\n"));
}

void SerialBus::writeDebug(const std::string& line) {
    std::lock_guard<std::mutex> lock(writeMutex);
    asio::write(port, asio::buffer("D;" + line + "\n"));
}
