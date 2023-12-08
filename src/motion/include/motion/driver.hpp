#ifndef DRIVER_HPP
#define DRIVER_HPP
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "telebot_interfaces/msg/motor_goal.hpp"
#include "telebot_interfaces/msg/motor_goal_list.hpp"
#include <exception>
#include <memory>
#include <chrono>
#include <unistd.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
// #include "node_utils/node_utils.hpp"
#include <functional>
#include "telebot_interfaces/msg/motor_state_list.hpp"

using std::placeholders::_1;
using telebot_interfaces::msg::MotorGoal;
using telebot_interfaces::msg::MotorGoalList;
using telebot_interfaces::msg::MotorStateList;
// template <typename SubscriptionType,typename PublishType>

/// Base Class for any motor driver to derive from. Provides synchronization and buffering of messages.
class MotorDriver{
public:
    MotorDriver(){}
    /// @brief Writes to motors and then reads status of motors.
    /// @param motorVals The values to write to the motors
    /// @return The current state of the robots motors
    MotorStateList motorWriteRead(){
        _writeMotors();
        return _readMotors();
    }
    void setWriteValues(const MotorGoalList& msg){
        //While the motors are writing
        while (_writeLocked){
            sleep(0.05);//Arbitrary short wait time, adjust if it is too little. Ideally we are hitting wait maybe once per cycle, but not waisting time.
        }
        _buffer=msg;
    }
    virtual int initialize()=0;
    std::chrono::duration<int64_t, std::milli> getWriteFrequency()const{return motorWriteFrequency;}
    std::chrono::duration<int64_t, std::milli> getReadFrequency()const{return motorReadFrequency;}
    const MotorGoalList* getMotorGoals()const{return &_buffer;}
protected:
    /// @brief The delay between write loops. Intended to be used by a timer 
    std::chrono::duration<int64_t, std::milli> motorWriteFrequency=std::chrono::milliseconds(500);
    std::chrono::duration<int64_t, std::milli> motorReadFrequency=std::chrono::milliseconds(500);
    /// @brief Specific implementation of writing to the motors
    /// @param msg The goals to write
    virtual void writeMotors()=0;
    //Port read
    virtual MotorStateList readMotors()=0;
private:
    MotorStateList _readMotors(){
        auto tmp=readMotors();
        _activePortAccess=false;
        return tmp;
    }
    void _writeMotors(){
        _activePortAccess=true;
        _writeLocked=true;
        writeMotors();
        _writeLocked=false;
    }
    MotorGoalList _buffer;
    bool _writeLocked=false;
    bool _activePortAccess=false;
};

#endif