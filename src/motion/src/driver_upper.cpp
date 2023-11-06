#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "telebot_interfaces/msg/motor_goals.hpp"
#include <exception>
#include <memory>
using rclcpp::Node;
using telebot_interfaces::msg::MotorGoals;

class UpperDriver : public Node{
public:
    UpperDriver():Node("driver_upper"){
        rclcpp::QoS subQos(rclcpp::KeepLast(1)); // Setting queue size
        subQos.best_effort();                       // Setting communication to reliable. All messages will be received, publishers to this topic must also be reliable.
        subQos.durability_volatile(); // This means that this topic will only start getting messages once it is up.
        auto goalListener=this->create_subscription("goals/upper",)
    }
private:
    rclcpp::Publisher<MotorGoals>::SharedPtr _publisher;
    //Motor write loop timer
    rclcpp::TimerBase::SharedPtr _timer;
};


int main(int argc, char *argv[]){
    // Initialize the ROS2 system
    rclcpp::init(argc, argv);

    // Instantiate muxer
    auto driver = std::make_shared<UpperDriver>();
    // Execute until shutdown
    rclcpp::spin(driver);
    // Shut down the ROS2 system
    rclcpp::shutdown();
    return 0;
}