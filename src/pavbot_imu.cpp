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


class PavbotIMU : public rclcpp::Node
{
  private:

  // CLASS MEMBERS ----------------------------

  std::string port; // the port that the nano is connected to
  int baud; //baud rate for serial communication
  int timeout_ms;

  // declare the publishers here
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr imu_pub;

  //delcare all timers here
  rclcpp::TimerBase::SharedPtr timer;
  
  // Heading to publish
  double heading;

  // Function using in wall timer to update at a constant hertz
  void update() {
    // Connect to IMU
    // Read serial data
    // Process data
    // Format for publish

    std_msgs::msg::Float32 msg;

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
    port = declare_parameter<std::string>("port", "/dev/ttyUSB0"); // This is is the Arduino is in the USB0 slot it's just a placeholder
    baud = declare_parameter<int>("baud", 115200);
    timeout_ms = declare_parameter<int>("timeout_ms", 500);

    // Publishers ------------------
    imu_pub = create_publisher<std_msgs::msg::Bool>("/sensors/imu", rclcpp::QoS(1).transient_local() // a queue depth of 1 to keep the latest value :)
    );


    // timers ----------------
    timer = create_wall_timer( std::chrono::milliseconds(50), std::bind(&PavbotIMU::update, this) // periodic timer for constant update
    );


    // Logging to the screen when the node is created to make sure it is there :)
    RCLCPP_INFO(get_logger(), "IMU node started"); // log that the node started 
    RCLCPP_INFO(get_logger(), "Chatbot, make me a hoot chocolate!"); // fun message
  }


};


int main(int argc, char **argv) {
// SPINNING UP THE ROS NODE
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PavbotIMU>());
  rclcpp::shutdown();
  return 0;
}
