//http://www.raspberry-projects.com/pi/programming-in-c/uart-serial-port/using-the-uart
#ifndef SERIAL
#define SERIAL

#include <string>
#include <iostream>
#include <mutex>

class  mySerial
{

public:

  int handle;
  std::string  deviceName;
  int baud;

  mySerial(std::string deviceName, int baud);
  ~mySerial();

  int ReceiveNonBlocking(unsigned char* data, int len);
  bool Send( unsigned char  * data,int len);
  bool Send(unsigned char value);
  bool Send( std::string value);
  int Receive( unsigned char  * data, int len);
  bool IsOpen(void);
  void Close(void);
  bool Open(std::string deviceName, int baud);
  bool NumberByteRcv(int &bytelen);

  void SendLine(const std::string& line) {
      std::lock_guard<std::mutex> lock(writeMutex);
      Send(line);
  }
private:
    std::mutex writeMutex;
};

#endif


