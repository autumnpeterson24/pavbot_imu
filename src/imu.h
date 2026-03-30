// O== Metadata ===================================================
// | Author: Michael Stalford for PAVBot Capstone Team, 2026
// | Purpose: Act as interfacing class to get navigation data from
// |          the Yost 3-Space IMU v1.0 using ASCII commands.
// | Version:
// |   3/26/26     Michael Stalford     Born anew to allow for all
// |                                    modes beyond heading to be
// |                                    collected
// O== To-Do ======================================================
// | 1. Make sure that x = forward, y = left, an z = up
// | 2. Implement the three factors of position reckoning above
// |    - Alternatively, sleep and George Orr it into existence (?)
// |      (not likely to work)
// | 3. Change defaults to be static port stuff not dev path
// O== Notes ======================================================
// | Yost 3-Space IMU v1.0 ASCII Commands (linked to Autumn's req)
// | FWD: +x      LEFT: +y     UP: +z
// |---------------------------------------------------------------
// |  1. Heading           0-356 degrees CW from true North
// |
// |    :26,1\n    ->    Enable compass. Must be done on init so
// |                     compass becomes active.
// |    :01\n      ->    Get filtered Euler angles as:
// |                      "pitch,yaw,roll\r\n"     (degrees)
// |
// |  2. Orientation       Quaternion (x, y, z, w)
// |  
// |    :06\n      ->    Get quaternion as:
// |                      "x,y,z,w\r\n"            (unit quat)
// |
// |  3. Angular Velocity  rad/s, ASSUMED CW
// |  
// |    :37\n      ->    Gyro rate as:
// |                      "x,y,z\r\n"              (rad/s)
// |
// O===============================================================
#pragma once

// O== Imports ====================================================
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <vector>
#include <cstring>
#include <iostream>
#include <cerrno>
#include <utility>
#include <optional>
#include <sstream>
#include <cmath>

// O== Tools o' The Trade =========================================
struct Quaternion {double x, y, z, w; };
struct Vector3    {double x, y, z; };
struct IMUPacket  {
    double      heading;
    Quaternion  orientation;
    Vector3     angVel;
}; 

// O== Class ======================================================
class IMU {
  //-- Public -----------------------------------------------------
  public:
    //-- Set-up ---------------------------------------------------
    IMU(std::string port = "/dev/ttyUSB0",
        speed_t baud =     B115200,
        int timeout_ms =   500,
        double remap_x =   1.0, // Using this scheme should allow
        double remap_y =  -1.0, // us to map our directions w/o
        double remap_z =  -1.0) // fixing a bunch of math later
      : port(port), baud(baud), timeout_ms(timeout_ms),
        rx(remap_x), ry(remap_y), rz(remap_z), fd(-1) {
            
        }

    void configure(std::string& port, speed_t baud, int timeout_ms) {
        this->port = port;
        this->baud = baud;
        this->timeout_ms = timeout_ms;
    }

    //-- Data Interaction -----------------------------------------
    bool open() {
        fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) return false;

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
        tty.c_oflag &= ~OPOST;

        // Non-blocking
        tty.c_cc[VMIN] = 0; // 0 byte casew
        tty.c_cc[VTIME] = timeout_ms / 100;

        tcsetattr(fd, TCSANOW, &tty);
        return true;
    }
    
    bool close() {
        return false;
    }
    
    void flush() {
        tcflush(this->fd, TCIFLUSH);
    }
    
    std::optional<std::string> readLine() {
        char c;

        while (true) {
            ssize_t n = ::read(this->fd, &c, 1);
            if (n <= 0) return std::nullopt;
            if (c == '\r') continue;
            if (c == '\n') {
                std::string line = rxBuf;
                rxBuf.clear();
                return line;
            }

            rxBuf.push_back(c);
        }
    }
    
    //-- Heading --------------------------------------------------
    bool enableCompass() { return writeCommand(":26,1\n"); }
    bool queryHeading() { flush(); return writeCommand(":40\n"); }

    // Returns degrees [0, 360) CW from true N
    std::optional<double> readHeading() {
        // Query data and if it fails return nothing
        if (!queryHeading()) return std::nullopt;
        
        // Read recv'd data and if it fails return nothing
        auto line = readLine();
        if (!line) return std::nullopt;

        // Some packet parsing lexer-style
        std::istringstream ss(*line);
        std::string tok;

        // Get the data
        double pitch{}, yaw{}, roll{};
        if (!std::getline(ss, tok, ',')) return std::nullopt;
        try { roll = std::stod(tok); } catch (...) { return std::nullopt; }
 
        if (!std::getline(ss, tok, ',')) return std::nullopt;
        try { pitch = std::stod(tok); } catch (...) { return std::nullopt; }
 
        if (!std::getline(ss, tok, ',')) return std::nullopt;
        try { yaw = std::stod(tok); } catch (...) { return std::nullopt; }
    
        // Not sure if this quite works-- check here for issues
        // yaw = std::fmod(yaw*57, 360.0);
        // if (yaw< 0.0) yaw += 360.0;
        double heading = (yaw + 0.6) * 600 - 60;
        return heading; 
    }
    
    //-- Position -------------------------------------------------
    bool queryPosition() { flush(); return writeCommand(":06\n"); }
        
    std::optional<Quaternion> readPosition() {
        // Query data and if it fails return nothing
        if (!queryPosition()) return std::nullopt;
        
        // Read recv'd data and if it fails return nothing
        auto line = readLine();
        if (!line) return std::nullopt;
        
        // Some packet parsing lexer-style
        std::istringstream ss(*line);
        std::string tok;
        
        // Get the data
        double vals[4];
        for (int i = 0; i < 4; ++i) {
            if (!std::getline(ss, tok, ',')) return std::nullopt;
            try { vals[i] = std::stod(tok); } catch (...) { return std::nullopt; }
        }
        
        // I have not verified this but should work for remapping
        Quaternion q;
        q.x = rx * vals[0];
        q.y = ry * vals[1];
        q.z = rz * vals[2];
        q.w =       vals[3];
        return q;
    }
    
    //-- Velocity -------------------------------------------------
    bool queryVelocity() { flush(); return writeCommand(":37\n"); }
        
    std::optional<Vector3> readVelocity() {
        // Query data and if it fails return nothing
        if (!queryVelocity()) return std::nullopt;
        
        // Read recv'd data and if it fails return nothing
        auto line = readLine();
        if (!line) return std::nullopt;
        
        // Some packet parsing lexer-style
        std::istringstream ss(*line);
        std::string tok;
        
        // Get the data
        double vals[3];
        for (int i = 0; i < 3; ++i) {
            if (!std::getline(ss, tok, ',')) return std::nullopt;
            try { vals[i] = std::stod(tok); } catch (...) { return std::nullopt; }
        }
        
        return Vector3{ rx * vals[0], ry * vals[1], rz * vals[2] };
    }
    
    // Synthesize, baby! ------------------------------------------
    std::optional<IMUPacket> readAll() {
        auto h = readHeading();
        if (!h) return std::nullopt;
        
        auto p = readPosition();
        if (!p) return std::nullopt;
        
        auto v = readVelocity();
        if (!v) return std::nullopt;
        
        return IMUPacket{ *h, *p, *v };
    }
    
  //-- Private Data -----------------------------------------------
  private:
    // Ports
    std::string port;
    speed_t baud;
    int timeout_ms;
    
    // Adjustments
    double rx, ry, rz;
    
    // Management
    int fd;
    std::string rxBuf;
    

    bool writeCommand(std::string c) {
        ssize_t n = ::write(this->fd, c.c_str(), c.size());
        tcdrain(fd);
        return n == static_cast<ssize_t>(c.size());
    }
};