#include <memory>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include "kinematics/KinematicsClass.h"  // Make sure this include path is correct

#define MOTOR_MAX_RPM 90        // Motor's maximum RPM
#define WHEEL_DIAMETER 0.2      // Robot's wheel diameter in meters
#define FR_WHEEL_DISTANCE 0.6   // Front-rear wheel distance in meters
#define LR_WHEEL_DISTANCE 0.5   // Left-right wheel distance in meters
#define PWM_BITS 8              // PWM resolution bits

class KinematicsNode : public rclcpp::Node
{
public:
  KinematicsNode()
  : Node("kinematics_node"),
    kinematics_(MOTOR_MAX_RPM, WHEEL_DIAMETER, FR_WHEEL_DISTANCE, LR_WHEEL_DISTANCE, PWM_BITS)
  {
    RCLCPP_INFO(this->get_logger(), "Kinematics Node Initialized");
    // Create a subscription to the cmd_vel topic
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&KinematicsNode::rpm_callback, this, std::placeholders::_1));
      motor_rpm_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("motor_rpm", 10);

  }

private:
  // The callback uses the incoming Twist message to calculate the RPM for each motor.
  void rpm_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Calculate motor RPMs using the linear and angular velocities from the message.
    auto rpm = kinematics_.getRPM(msg->linear.x, msg->linear.y, msg->angular.z);

    // Create rpm_msg and publish to motor_rpm topic
    std_msgs::msg::Int32MultiArray rpm_msg;
    rpm_msg.data = {rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4};
    motor_rpm_pub_->publish(rpm_msg);

    // Log the calculated motor RPM values.
    RCLCPP_INFO(
      this->get_logger(),
      "Motor RPM: FrontR: %d, FrontL: %d, BackR: %d, BackL: %d",
      rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4);
  }

  // Subscription to the "cmd_vel" topic.
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  // Publisher to "motor_rpm" topic
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr motor_rpm_pub_;
  // Kinematics instance created with the specified parameters.
  KINEMATICS::Kinematics kinematics_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
