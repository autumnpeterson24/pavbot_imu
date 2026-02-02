// Forgot exact syntax for import guards... will implement later

#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <cerrno>
#include <utility>

class IMU {
  public:
    IMU(std::string port = "/dev/ttyUSB0", speed_t baud = B115200, int timeout_ms = 500)
      : port(port), baud(baud), timeout_ms(timeout_ms) {
        fd = -1;
    }

    void configure(std::string port, speed_t baud, int timeout_ms) {
        this->port = port;
        this->baud = baud;
        this->timeout_ms = timeout_ms;
    }

    bool open() {
        fd = ::open(port.c_str(), O_RDONLY | O_NOCTTY);
        if (fd < 0) {
            return false;
        } 

        struct termios tty{};
        tcgetattr(fd, &tty);

        cfsetospeed(&tty, baud);
        cfsetispeed(&tty, baud);

        // For UART 8N1
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;

        // Raw mode (?)
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_lflag &= ~(IXON | IXOFF | IXANY);
        tty.c_lflag &= ~OPOST;

        // Non-blocking
        tty.c_cc[VMIN] = 0; // 0 byte casew
        tty.c_cc[VTIME] = timeout_ms / 100; // 1 second?

        tcsetattr(fd, TCSANOW, &tty);
        return true;
    }

    std::string readSerial() {
        char buf[256];
    
        int n = read(this->fd, buf, sizeof(buf) - 1);
        if (n <= 0) {
            return {};
        }

        buf[n] = '\0';
        return std::string(buf);
    }

    int readHeading() {
        // Returns the heading as degrees
        // counter-clockwise from north
    }

    bool close() {
        return false;
    }

  private:
    int fd;
    std::string port;
    speed_t baud;
    int timeout_ms;
};