#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "telebot_interfaces/msg/motor_goal_list.hpp"
#include "telebot_interfaces/msg/motor_state.hpp"
#include <exception>
#include <memory>
#include <unordered_map>
#include <list>
#include <mutex>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "motion/dynamixel_addresses.hpp"
#include "motion/topic_prefixes.hpp"
#include "motion/dynamixel_driver.hpp"
#include "motion/motion.hpp"

using rclcpp::Node;
using telebot_interfaces::msg::MotorGoalList;
using telebot_interfaces::msg::MotorState;
using Motion::MovementType;

class LowerDriverNode : public Node
{
public:
    LowerDriverNode() : Node("driver_lower")
    {
        decParams();
        initDriver();
        
        rclcpp::QoS subQos(3);
        
        _subscriber = this->create_subscription<MotorGoalList>(TopicPrefixes::getPrivateTopicName("motor_goals"), subQos, std::bind(&LowerDriverNode::listenGoals, this, _1));
        _publisher = this->create_publisher<MotorStateList>(TopicPrefixes::getPublicTopicName("lower_state"), subQos);
        _driverTimer = this->create_wall_timer(_driver->getWriteFrequency(), std::bind(&LowerDriverNode::motorWriteRead, this));
        RCLCPP_INFO(this->get_logger(),"Lower driver successfully started...");
    }

private:
    void initDriver(){
        _driver= std::make_shared<DynamixelDriver>(get_parameter("usb_device").as_string().c_str(),get_parameter("usb_baudrate").as_int());
        _driver->setProMotors(get_parameter("pro_motors").as_integer_array());
        if (_driver->initialize() == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Driver initialization failed!!! Driver not running!!! Check cerr output for more info.");
            throw std::system_error();
            return;
        }
        RCLCPP_INFO(this->get_logger(),"Driver initialization successful!");
        _driver->setTorqueAllMotors(true);
        RCLCPP_INFO(this->get_logger(),"Motors Torqued!");
    }
    void decParams(){
        this->declare_parameter("pro_motors", std::vector<int>{});
        this->declare_parameter("roboclaw_motors", std::vector<int>{});
        this->declare_parameter("usb_device", "DEFAULT");
        this->declare_parameter("usb_baudrate", 1000000);
    }
    void listenGoals(const MotorGoalList &msg)
    {
        _driver->setWriteValues(msg);
    }
    void motorWriteRead()
    {
        auto msg=_driver->motorWriteRead();
        if(msg.motor_states.size()==0){
            RCLCPP_WARN(this->get_logger(),"No values read from motor. It is possible power was lost or a connection is loose.");
        }
        if(msg.motor_states.size()<_driver->motorCount()){
            RCLCPP_WARN(this->get_logger(),"Only read from %d/%d motors. A connection could be loose...",msg.motor_states.size(),_driver->motorCount());
        }
        _publisher.get()->publish(msg);
    }
    std::shared_ptr<DynamixelDriver> _driver;
    rclcpp::TimerBase::SharedPtr _driverTimer;
    rclcpp::Publisher<MotorStateList>::SharedPtr _publisher;
    rclcpp::Subscription<MotorGoalList>::SharedPtr _subscriber;
};

int main(int argc, char *argv[])
{
    // Initialize the ROS2 system
    rclcpp::init(argc, argv);
    try
    {
        // Instantiate muxer
        auto driver = std::make_shared<LowerDriverNode>();
        // Execute until shutdown
        rclcpp::spin(driver);
    }
    catch (std::system_error e)
    {
        std::cout << "Exiting...";
    }
    // Shut down the ROS2 system
    rclcpp::shutdown();
    return 0;
}