#ifndef DRIVER_HPP
#define DRIVER_HPP
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "telebot_interfaces/msg/motor_goal.hpp"
#include "telebot_interfaces/msg/motor_goal_list.hpp"
#include <exception>
#include <memory>
#include "dynamixel_sdk/dynamixel_sdk.h"
// #include "node_utils/node_utils.hpp"
#include "telebot_interfaces/msg/motor_state_list.hpp"
using rclcpp::Node;
using rclcpp::QoS;
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
    MotorStateList motorWriteRead(MotorGoalList& motorVals){
        _writeMotors(motorVals);
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
protected:
    /// @brief The delay between write loops. Intended to be used by a timer 
    std::chrono::duration<int64_t, std::milli> motorWriteFrequency=std::chrono::milliseconds(500);
    std::chrono::duration<int64_t, std::milli> motorReadFrequency=std::chrono::milliseconds(500);
    /// @brief Specific implementation of writing to the motors
    /// @param msg The goals to write
    virtual void writeMotors(const MotorGoalList& msg)const=0;
    //Port read
    virtual MotorStateList readMotors()=0;
private:
    MotorStateList _readMotors(){
        auto tmp=readMotors();
        _activePortAccess=false;
        return tmp;
    }
    void _writeMotors(const MotorGoalList& motorVals){
        _activePortAccess=true;
        _writeLocked=true;
        writeMotors(motorVals);
        _writeLocked=false;
    }
    MotorGoalList _buffer;
    bool _writeLocked=false;
    bool _activePortAccess=false;
};

// template <typename SubscriptionType,typename PublishType>
// class DriverNode: public Node{
// public:
//     DriverNode(const char* nodeName,const char* motorGoalSubTopic,const char* motorStatePubTopic,void*(void)(SubscriptionType)):Node(nodeName){
//         //Init publisher
//         QoS pubQos(10);
//         _publisher=this->create_publisher<PublishType>(motorStatePubTopic,pubQos);

//         //Init subscriber
//         QoS subQos(rclcpp::KeepLast(2)); // Setting queue size
//         subQos.best_effort();          // Makes it so that it is not guaranteed that a message doesn't get dropped, driver will not block for a message.
//         subQos.durability_volatile(); // This means that this topic will only start getting messages once it is up.
//         auto _subscriber=this->create_subscription(motorGoalSubTopic,subQos,std::bind(subscriptionCallback, this, _1));//Subscribe to receive the write messages.
//     }

// protected:
//     virtual void writeMotors()=0;
//     virtual void listenGoals(SubscriptionType& msg)=0;
//     typename rclcpp::Publisher<PublishType>::SharedPtr _publisher;
//     rclcpp::TimerBase::SharedPtr _timer;
//     bool _writingMotorsLock=false;
// };

#endif