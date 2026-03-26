// O== Metadata ===================================================
// | Author: Michael Stalford for PAVBot Capstone Team, 2026
// | Purpose: Act as interfacing class to get navigation data from
// |          the Yost 3-Space IMU v1.0 using ASCII commands.
// | Version:
// |   3/26/26     Michael Stalford     Born anew to allow for all
// |                                    modes beyond heading to be
// |                                    collected
// O== To-Do ======================================================
// |
// O== Notes ======================================================
// | Subscribes to: 
// | - Nothing
// | 
// | Publishes:
// | - sensors/imu/heading: Float32
// |                        [0, 360) CW from true North
// | - ??? CHECK WITH AUTUMN FOR HOW SHE NEEDS THIS DONE!!!
// |                        
// O===============================================================
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <math.h>

// For IMU
#include "imu.h"

#define DEG_OFFSET 0

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
    IMU imu;

  // Function using in wall timer to update at a constant hertz
  void update() {
    auto data = imu.readAll();
    if (!data) return;

    // Heading
    std_msgs::msg::Float32 headingMsg;
    headingMsg.data = static_cast<float>(data->heading);
    RCLCPP_INFO(get_logger(), "Heading: %.1f deg", headingMsg.data);
    imu_pub->publish(headingMsg);
    
    // Others ??? ASK AUTUMN FORMAT AND NEED TO DO SOME PROCESSING MYSELF!!!
  }

public:
// CONSTRUCTOR -----------------------------
  PavbotIMU() : Node("pavbot_imu") {
    // Parameters -----------------
    port = declare_parameter<std::string>("port", "/dev/ttyUSB0"); 
    //baud = declare_parameter<int>("baud", 115200);
    timeout_ms = declare_parameter<int>("timeout_ms", 500);

    // Publishers ------------------
    imu_pub = create_publisher<std_msgs::msg::Float32>("/sensors/imu/heading", rclcpp::SensorDataQoS());
    //??

    // timers ----------------
    timer = create_wall_timer( std::chrono::milliseconds(50), std::bind(&PavbotIMU::update, this) // periodic timer for constant update
    );

    // Logging to the screen when the node is created to make sure it is there :)
    RCLCPP_INFO(get_logger(), "IMU node started"); // log that the node started 

    // IMU Stuff
    imu.configure(port, B115200, timeout_ms);

    if (imu.open()) {
      // Success
      RCLCPP_INFO(get_logger(), "Successful connection to IMU. Awaiting data packets...");
      imu.enableCompass();
    } else {
      // Failure
      RCLCPP_INFO(get_logger(), "Failure to connect to IMU :: %s", strerror(errno));
    }
  }
};

int main(int argc, char **argv) {
// SPINNING UP THE ROS NODE
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PavbotIMU>());
  rclcpp::shutdown();
  return 0;
}
