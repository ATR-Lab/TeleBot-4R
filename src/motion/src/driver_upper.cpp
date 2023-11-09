#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "telebot_interfaces/msg/motor_goal_list.hpp"
#include "telebot_interfaces/msg/motor_state.hpp"
#include <exception>
#include <memory>
#include <unordered_map>
#include <list>
#include <mutex>
#include "motion/driver.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
using rclcpp::Node;
using telebot_interfaces::msg::MotorGoalList;
using telebot_interfaces::msg::MotorState;
enum MovementType{
    POSITION='P',PWM='T',VELOCITY='V'
}
class UpperDriver : public MotorDriver
{
public:
    UpperDriver(const char *device_name,int baudrate ,const float protocol_version = 2.0F) : MotorDriver(),
    _device_name(device_name),_baudrate(baudrate),_protocol_version(protocol_version){}
    int initialize(){
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
        dxl_comm_result = _portHandler->setBaudRate(_baudrate);
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
    void writeMotors(const MotorGoalList &msg) const
    {
        auto bulkWriter=dynamixel::GroupBulkWrite(_portHandler,_packetHandler);
        for(auto&goal:msg.motor_goals){
            
            bulkWriter.addParam(goal.motor_id,);goal.
        }
        //Write to motors
        bulkWriter.txPacket();
    }
    // Port read
    MotorStateList readMotors()
    {
        // MotorStateList
    }

private:
    dynamixel::PortHandler *_portHandler;
    dynamixel::PacketHandler *_packetHandler;

    //Parameters that should be gotten from a node
    const char *_device_name;
    int _baudrate;
    float _protocol_version = 2.0F;

    std::pair<uint16_t,uint16_t> getModeInfo(const MotorGoal& goal){
        switch (goal.movement_type[0])
        {
        case MovementType::POSITION:
            if(isProMotor(goal.motor_id)){
                const auto POS_GOAL_ADDRESS_PRO=1;
                const auto POS_GOAL_ADDRESS_DXL=1;
                return 
            }
            
            break;
        case MovementType::PWM:
            /* code */
            break;
        case MovementType::VELOCITY:
            /* code */
            break;
        default:
            return std::make_pair(UINT16_MAX,UINT16_MAX);
        }

    }

    bool isProMotor(const int16_t id)const{
        const std::vector<int> proMotors={4,5,11,12,30,31};
        for(auto val:proMotors){
            if(id==val){
                return true;
            }
        }
        return false;
    }
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