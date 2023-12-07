#ifndef DYNAMIXEL_DRIVER_HPP
#define DYNAMIXEL_DRIVER_HPP
#include "motion/driver.hpp"
#include "motion/dynamixel_addresses.hpp"
#include "motion/motion.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
using Motion::MovementType;
class DynamixelDriver : public MotorDriver
{
public:
    
    DynamixelDriver():MotorDriver(){}
    // const std::vector<int> motorIDS={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,};
    DynamixelDriver(const char *device_name, int baudrate, const float protocol_version = 2.0F) : MotorDriver(),
                                                                                              _device_name(device_name), _baudrate(baudrate), _protocol_version(protocol_version) {}
    int initialize()
    {
        // Device name should be gotten from params
        // Instantiate our port and packet handlers
        _portHandler = dynamixel::PortHandler::getPortHandler(_device_name);            // eg. ttyUSB0;
        _packetHandler = dynamixel::PacketHandler::getPacketHandler(_protocol_version); // eg. 1 or 2
        auto res=initPortHandler();
        _bulkWriter=std::make_unique<dynamixel::GroupBulkWrite>(dynamixel::GroupBulkWrite(_portHandler, _packetHandler));
        return res;
    }
    int initPortHandler(){
        auto dxl_comm_result = _portHandler->openPort();
        if (dxl_comm_result == false)
        {

            std::cerr << "\033[31mFailed to open port \"" << _device_name << "\"!\n\033[0m";

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
            std::cerr << "\033[31mFailed to set the baudrate! " << _baudrate << "\n\033[0m";
            return -1;
        }
        else
        {
            std::cout << "Succeeded to set the baudrate.\n";
        }
        return 0;
    }
    /// @brief Torques all motors, or a specified vector of motors
    /// @param ids The id vector of motors to torque
    void torqueMotor(int id, bool torqued){
        int address=isProMotor(id)?DynamixelAddresses::Pro::RAM::TORQUE_ENABLE_ADDR:DynamixelAddresses::XM540::RAM::TORQUE_ENABLE_ADDR;
        _packetHandler->write1ByteTxOnly(_portHandler,id,address,torqued);
    }

    void setTorqueAllMotors(bool torqued){
        uint8_t torqueInt=torqued;
        uint8_t* data= &torqueInt;
        for(auto&id:_proMotors){
            using namespace DynamixelAddresses::Pro::RAM;
            _bulkWriter->addParam(id,TORQUE_ENABLE_ADDR,TORQUE_ENABLE_SIZE,data);
        }
        for(auto&id:_xmMotors){
            using namespace DynamixelAddresses::XM540::RAM;
            _bulkWriter->addParam(id,TORQUE_ENABLE_ADDR,TORQUE_ENABLE_SIZE,data);
        }
        _bulkWriter->txPacket();
        _bulkWriter->clearParam();
    }
    void setProMotors(const std::vector<int64_t>& ids){
        _proMotors=ids;
    }
    void setXmMotors(const std::vector<int64_t>& ids){
        _xmMotors=ids;
    }
    auto getMotorIDs(){
        auto ret=_xmMotors;
        ret.insert(ret.end(),_proMotors.begin(),_proMotors.end());
        return ret;
    }
    int motorCount(){
        return _xmMotors.size()+_proMotors.size();
    }
    // ~DynamixelDriver()
    // {
    //     _portHandler->clearPort();
    //     _portHandler->closePort();
    // }
protected:
    
    void writeMotors()
    {
        auto goals = getMotorGoals();
        // A sync writer would likely be more efficient, and easier to implement, need to guarantee that we will only be doing position writes
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
                _bulkWriter->addParam(goal.motor_id, addr, size, data);
            }
            catch (std::logic_error e)
            {
                std::cerr << e.what() << "\n";
            }

            data = nullptr; // Point off to nowhere so it doesn't clean up transformed vals twice
        }
        // Write to motors
        _bulkWriter->txPacket();
        _bulkWriter->clearParam();
    }
    // Port read
    MotorStateList readMotors()
    {        
        MotorStateList list;
        dynami
        return list;
    }

private:
    std::vector<int64_t> _proMotors;
    std::vector<int64_t> _xmMotors;
    dynamixel::PortHandler *_portHandler;
    dynamixel::PacketHandler *_packetHandler;
    std::unique_ptr<dynamixel::GroupBulkWrite> _bulkWriter;
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

        
        for (const auto &val : _proMotors)
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
#endif