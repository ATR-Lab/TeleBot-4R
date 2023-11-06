#ifndef DRIVER_HPP
#define DRIVER_HPP
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "telebot_interfaces/msg/motor_goals.hpp"
#include <exception>
#include <memory>
#include "dynamixel_sdk.h"
using rclcpp::Node;
class Driver: public Node{
public:
    Driver(const char* nodeName):Node(nodeName){};
    virtual void writeMotors()=0;
private:
    rclcpp::Publisher<>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
};

#endif