#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "telebot_interfaces/msg/motor_goals.hpp"
#include "telebot_interfaces/msg/motor_state.hpp"
#include <exception>
#include <memory>
#include <unordered_map>
#include <list>
#include <mutex>
#include "motion/driver.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
using rclcpp::Node;
using telebot_interfaces::msg::MotorGoals;
using telebot_interfaces::msg::MotorState;
// using dynamixel::GroupBulkWrite;
typedef std::list<rclcpp::Subscription<MotorGoals>> SubscriberList;
using std::placeholders::_1;

// class Driver2:public Driver<MotorGoals,MotorState>{
//     Driver2():Driver("driver_upper","tmp","out"){
//         _publisher
//     }
// };
class UpperDriver : public MotorDriver
{
public:
    UpperDriver(const char *device_name,int baudrate ,const float protocol_version = 2.0F) : MotorDriver(),
    _device_name(device_name),_baudrate(baudrate),_protocol_version(protocol_version){}
    virtual int initialize(){
        // Device name should be gotten from params
        // Instantiate our port and packet handlers
        _portHandler = dynamixel::PortHandler::getPortHandler(_device_name);            // eg. ttyUSB0;
        _packetHandler = dynamixel::PacketHandler::getPacketHandler(_protocol_version); // eg. 1 or 2

        auto dxl_comm_result = _portHandler->openPort();
        if (dxl_comm_result == false)
        {
            std::cerr<< "Failed to open the port! %s",_device_name;
            return -1;
        }
        else
        {
            std::cout<< "Succeeded to open the port.";
        }

        // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
        auto dxl_comm_result = _portHandler->setBaudRate(_baudrate);
        if (dxl_comm_result == false)
        {
            std::cerr<< "Failed to set the baudrate! %s",_baudrate;
            return -1;
            
        }
        else
        {
            std::cout<<"Succeeded to set the baudrate.";
        }
    }
    ~UpperDriver(){
        _portHandler->clearPort();
        _portHandler->closePort();
    }
protected:
    virtual void  writeMotors(const MotorGoalList &msg) const
    {
    }
    // Port read
    virtual MotorStateList readMotors()
    {
    }

private:

    dynamixel::PortHandler *_portHandler;
    dynamixel::PacketHandler *_packetHandler;

    //Parameters that should be gotten from a node
    const char *_device_name;
    int _baudrate;
    float _protocol_version = 2.0F;
};

int main(int argc, char *argv[])
{
    // Initialize the ROS2 system
    rclcpp::init(argc, argv);

    // Instantiate muxer
    // auto driver = std::make_shared<UpperDriver>();
    // // Execute until shutdown
    // rclcpp::spin(driver);
    // Shut down the ROS2 system
    rclcpp::shutdown();
    return 0;
}