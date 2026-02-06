// Forgot exact syntax for import guards... will implement later

#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <cerrno>
#include <utility>
#include <optional>

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
        fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
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
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_lflag &= ~OPOST;

        // Non-blocking
        tty.c_cc[VMIN] = 0; // 0 byte casew
        tty.c_cc[VTIME] = timeout_ms / 100;

        tcsetattr(fd, TCSANOW, &tty);
        return true;
    }

    bool enableCompass() {
        return writeCommand(":26,1\n");
    }

    bool queryData() {
        tcflush(this->fd, TCIFLUSH);
        return writeCommand(":01\n");
    }

    std::optional<std::string> readLine() {
        static std::string buf;
        char c;

        while (true) {
            ssize_t n = ::read(this->fd, &c, 1);
            if (n <= 0) {
                return std::nullopt;
            }

            if (c == '\n') {
                std::string line = buf;
                buf.clear();
                return line;
            }

            buf.push_back(c);
        }
    }

    std::optional<double> readHeading() {
        auto line = readLine();
        if (!line) return std::nullopt;

        // Some packet parsing lexer-style
        std::stringstream ss(*line);
        std::string token;

        if (line->find("EULER") != std::string::npos) {
            std::getline(ss, token, ',');
        }

        double roll, pitch, yaw;

        if (!(std::getline(ss, token, ',') && (roll = std::stod(token), true))) return std::nullopt;
        if (!(std::getline(ss, token, ',') && (yaw = std::stod(token), true))) return std::nullopt;
        if (!(std::getline(ss, token, ',') && (pitch = std::stod(token), true))) return std::nullopt;
    
        return yaw;
    }

    bool close() {
        return false;
    }

  private:
    int fd;
    std::vector<uint8_t> rxBuf; // For stitching
    std::string port;
    speed_t baud;
    int timeout_ms;

    bool writeCommand(std::string c) {
        ssize_t n = ::write(this->fd, c.c_str(), c.size());
        tcdrain(fd);
        
        return n == static_cast<ssize_t>(c.size());
    }
};