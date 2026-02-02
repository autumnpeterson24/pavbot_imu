/*
E-Stop Node for Pavbot
author: Michael Stalford

Subscribes to:
 - Nothing

 Publishes:
  - sensors/imu/heading: Float32 as degrees between 0 and 359 COUNTER-clockwise from North
*/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

// For I/O with the IMU
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <cerrno>

class PavbotIMU : public rclcpp::Node
{
  private:

  // CLASS MEMBERS ----------------------------
  std::string port; // the port that the nano is connected to
  int baud; //baud rate for serial communication
  int timeout_ms;

  // declare the publishers here
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_pub;

  //delcare all timers here
  rclcpp::TimerBase::SharedPtr timer;
  
    // IMU
  int fd; // The port for interfacing (descriptor-style)
  char buf[256]; // The buffer to hold UART strings from the IMU

  // Heading to publish
  double heading;

  // Function using in wall timer to update at a constant hertz
  void update() {
    // Connect to IMU
    // Read serial data
    // Process data
    // Format for publish

    std_msgs::msg::Float32 msg;

    heading = 5;
    msg.data = static_cast<float>(heading);  
    imu_pub->publish(msg);

  }

  void getSerial() {
    // Magic
  }

public:
// CONSTRUCTOR -----------------------------
  PavbotIMU() : Node("pavbot_imu") {
    // Parameters -----------------
    port = declare_parameter<std::string>("port", "/dev/ttyUSB0"); 
    //baud = declare_parameter<int>("baud", 115200);
    //timeout_ms = declare_parameter<int>("timeout_ms", 500);

    // Publishers ------------------
    imu_pub = create_publisher<std_msgs::msg::Float32>("/sensors/imu", rclcpp::QoS(1).transient_local() // a queue depth of 1 to keep the latest value :)
    );


    // timers ----------------
    timer = create_wall_timer( std::chrono::milliseconds(500), std::bind(&PavbotIMU::update, this) // periodic timer for constant update
    );

    // Logging to the screen when the node is created to make sure it is there :)
    RCLCPP_INFO(get_logger(), "IMU node started"); // log that the node started 

    // Prep
    // IMU Prepping!
    fd = ::open(port.c_str(), O_RDONLY | O_NOCTTY | O_SYNC);
    if (fd < 0) {
      RCLCPP_INFO(get_logger(), "Failure to connect to IMU :: %s", strerror(errno));
    } else {
      RCLCPP_INFO(get_logger(), "Successful connection to IMU. Awaiting data packets...");
    }

    struct termios tty{};
    tcgetattr(fd, &tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

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
    tty.c_cc[VTIME] = 10; // 1 second?

    tcsetattr(fd, TCSANOW, &tty);
  }
};


int main(int argc, char **argv) {
// SPINNING UP THE ROS NODE
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PavbotIMU>());
  rclcpp::shutdown();
  return 0;
}
