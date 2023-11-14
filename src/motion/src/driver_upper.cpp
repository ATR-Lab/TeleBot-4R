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
#include "motion/dynamixel_addresses.hpp"
using rclcpp::Node;
using telebot_interfaces::msg::MotorGoalList;
using telebot_interfaces::msg::MotorState;

enum MovementType
{
    POSITION = 'P',
    PWM = 'T',
    VELOCITY = 'V'
};
class UpperDriver : public MotorDriver
{
public:
    UpperDriver(const char *device_name, int baudrate, const float protocol_version = 2.0F) : MotorDriver(),
                                                                                              _device_name(device_name), _baudrate(baudrate), _protocol_version(protocol_version) {}
    int initialize()
    {
        // Device name should be gotten from params
        // Instantiate our port and packet handlers
        _portHandler = dynamixel::PortHandler::getPortHandler(_device_name);            // eg. ttyUSB0;
        _packetHandler = dynamixel::PacketHandler::getPacketHandler(_protocol_version); // eg. 1 or 2

        auto dxl_comm_result = _portHandler->openPort();
        if (dxl_comm_result == false)
        {
            
            std::cerr << "\033[31mFailed to open port \""<< _device_name<<"\"!\n\033[0m";
            
            return -1;
        }
        else
        {
            std::cout << "Succeeded to open the port.\n";
        }

        // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
        dxl_comm_result = _portHandler->setBaudRate(_baudrate);
        if (dxl_comm_result == false)
        {
            std::cerr << "\033[31mFailed to set the baudrate! "<< _baudrate<<"\n\033[0m";
            return -1;
        }
        else
        {
            std::cout << "Succeeded to set the baudrate.\n";
        }
        return 0;
    }
    ~UpperDriver()
    {
        _portHandler->clearPort();
        _portHandler->closePort();
    }

protected:
    
    void writeMotors() const
    {
        auto goals=getMotorGoals();
        // A sync writer would likely be more efficient, and easier to implement, need to guarantee that we will only be doing position writes
        auto bulkWriter = dynamixel::GroupBulkWrite(_portHandler, _packetHandler);
        for (auto &goal : goals->motor_goals)
        {
            uint16_t addr, size;
            uint8_t *data;
            try
            {
                std::tie(addr, size) = getModeInfo(goal);
                int32_t transformedGoal = applyTransform(goal);
                // Convert to a uint_8 array
                const uint8_t MAX_BYTE_WRITE = 4;
                // THIS IS PROBABLY UNSAFE CODE!!!
                data = ((uint8_t *)(&transformedGoal)) + (MAX_BYTE_WRITE - size);
                bulkWriter.addParam(goal.motor_id, addr, size, data);
            }
            catch (std::logic_error e)
            {
                std::cerr << e.what() << "\n";
            }

            data = nullptr; // Point off to nowhere so it doesn't clean up transformed vals twice
        }
        // Write to motors
        bulkWriter.txPacket();
    }
    // Port read
    MotorStateList readMotors()
    {
        MotorStateList list;
        return list;
    }

private:
    dynamixel::PortHandler *_portHandler;
    dynamixel::PacketHandler *_packetHandler;

    // Parameters that should be gotten from a node
    const char *_device_name;
    int _baudrate;
    float _protocol_version = 2.0F;

    /// @brief Applies any required transforms based on motion type
    /// @param goal The motor goal msg
    /// @return Effectively a byte array of the value
    int32_t applyTransform(const MotorGoal &goal) const
    {
        switch (goal.movement_type[0])
        {
        case MovementType::POSITION:
        {
            return radiansToTicks(goal.motor_goal, isProMotor(goal.motor_id));
        }
        case MovementType::PWM:
        {
            std::logic_error e("Translation not yet implemented for PWM!");
            throw e;
        }
        case MovementType::VELOCITY:
        {
            std::logic_error e("Translation not yet implemented for VELOCITY!");
            break;
        }
        default:
        {
            std::invalid_argument e("Control mode not handled or doesn't exist!");
            throw e;
        }
        }
        return goal.motor_goal;
    }
    /// @brief Deduces the address and size for a given write
    /// @param goal The motor goal msg
    /// @return Pair of goal address, write size in bytes
    std::pair<uint16_t, uint16_t> getModeInfo(const MotorGoal &goal) const
    {
        using namespace DynamixelAddresses;
        std::pair<uint16_t, uint16_t> ret;
        uint16_t address = UINT16_MAX;
        uint16_t size = UINT16_MAX;
        switch (goal.movement_type[0])
        {
        case MovementType::POSITION:
        {
            if (isProMotor(goal.motor_id))
            {
                using namespace Pro::RAM;
                address = GOAL_POSITION_ADDR;
                size = GOAL_POSITION_SIZE;
            }
            else
            {
                using namespace XM540::RAM;
                address = GOAL_POSITION_ADDR;
                size = GOAL_POSITION_SIZE;
            }
            break;
        }
        case MovementType::PWM:
        {
            if (isProMotor(goal.motor_id))
            {
                using namespace Pro::RAM;
                address = GOAL_TORQUE_ADDR; // PWM and torque are the same, dif motors have dif names
                size = GOAL_TORQUE_SIZE;
            }
            else
            {
                using namespace XM540::RAM;
                address = GOAL_PWM_ADDR;
                size = GOAL_PWM_SIZE;
            }
            break;
        }
        case MovementType::VELOCITY:
        {
            if (isProMotor(goal.motor_id))
            {
                using namespace Pro::RAM;
                address = GOAL_VELOCITY_ADDR;
                size = GOAL_VELOCITY_SIZE;
            }
            else
            {
                using namespace XM540::RAM;
                address = GOAL_VELOCITY_ADDR;
                size = GOAL_VELOCITY_SIZE;
            }
            break;
        }
        default:
        {
            std::invalid_argument e("Control mode not handled or doesn't exist!");
            throw e;
        }
        }
        // Assign vals to ret
        ret.first = address;
        ret.second = size;

        return ret;
    }

    /// @brief Determines whether a given motor id is a pro motor
    /// @param id The motor id
    /// @return True if pro, false otherwise
    bool isProMotor(const uint16_t id) const
    {

        const std::vector<int> proMotors = {4, 5, 11, 12, 30, 31};
        for (const auto &val : proMotors)
        {
            if (id == val)
            {
                return true;
            }
        }
        return false;
    }

    /// @brief Translates a value from the range -3.14 to 3.14 into a tick value
    /// @param radians The value to translate in radians
    /// @return The value in ticks
    int32_t radiansToTicks(const float radians, const bool isPro) const
    {
        const auto PI = 3.14;
        const auto TWO_PI = 6.28;
        if (isPro)
        {
            const int32_t MAX_TICK_DIF = 501922;
            return (((radians + PI) / TWO_PI) * MAX_TICK_DIF) - (MAX_TICK_DIF / 2);
        }
        else
        {
            const int32_t MAX_TICK_DIF = 4095; // This is motor specific and can change, if you are getting weird write bugs this translation of ticks could be the issue.
            return ((radians + PI) / TWO_PI) * MAX_TICK_DIF;
        }
    }
};

class DriverNode:public Node{
    public:
    DriverNode():Node("driver"){
        
        if(_driver.initialize()==-1){
            RCLCPP_ERROR(this->get_logger(), "Driver initialization failed!!! Driver not running!!! Check cerr output for more info.");
            throw std::system_error();
            return;
        }
        rclcpp::QoS subQos(3);
        auto subscriber=this->create_subscription<MotorGoalList>("motor_goals",subQos,std::bind(&DriverNode::listenGoals,this, _1));
        _publisher=this->create_publisher<MotorStateList>("upper_state",subQos);
        _driverTimer=this->create_wall_timer(_driver.getWriteFrequency(),std::bind(&DriverNode::motorWriteRead, this));
    }
    private:
        void listenGoals(const MotorGoalList& msg){
            _driver.setWriteValues(msg);
        }
        void motorWriteRead(){
            _publisher.get()->publish(_driver.motorWriteRead());
        }
        UpperDriver _driver={"ttyUSB0",1000000};
        rclcpp::TimerBase::SharedPtr _driverTimer;
        rclcpp::Publisher<MotorStateList>::SharedPtr _publisher;

};



int main(int argc, char *argv[])
{
    // Initialize the ROS2 system
    rclcpp::init(argc,argv);
    try{
        // Instantiate muxer
        auto driver = std::make_shared<DriverNode>();
        // Execute until shutdown
        rclcpp::spin(driver);
    }
    catch(std::system_error e){
        std::cout<<"Exiting...";
    }
    // Shut down the ROS2 system
    rclcpp::shutdown();
    return 0;
}