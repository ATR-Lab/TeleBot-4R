/**
 * @file roboclaw_driver.hpp
 * @author Bailey Wimer (bwimer3@kent.edu)
 * @brief Adapts(or bridges?) the roboclaw driver from roboclaw_sdk
 *        into our customer driver class for use in the motion system
 * @version 0.1
 * @date 2024-01-02
 *
 * @copyright Copyright (c) 2024
 *
 */

// TODO: I might change the roboclaw driver that we are using later on,
//       but the interface shouldn't change at all.

#ifndef ROBOCLAW_DRIVER_HPP
#define ROBOCLAW_DRIVER_HPP

#define DEFAULT_ADDRESS 128
#include <iostream>
#include "node_utils/node_utils.hpp"
#include "motion/driver.hpp"
#include "motion/motion.hpp"

#include "roboclaw_sdk/driver.h"
#include "telebot_interfaces/msg/motor_state.h"
#include <exception>
#include <unordered_map>
#include <vector>
#include <string>
using telebot_interfaces::msg::MotorState;

class RoboclawDriver : public MotorDriver
{
public:
    RoboclawDriver() : MotorDriver() {}

    RoboclawDriver(std::pair<int,int> ids,const char *device_name, int baudrate, const float protocol_version = 2.0F) : MotorDriver(),
                                                                                                 _device_name(device_name), _baudrate(baudrate), _protocol_version(protocol_version),_ids(ids) {}
    int initialize() override
    {
        try
        {
            _driver = std::make_shared<driver>(NodeUtils::getPortBySerialID(_device_name).c_str(), _baudrate);
            // This might not be the best idea, not sure how it will play out...
            _driver->reset_encoders(DEFAULT_ADDRESS);
        }
        catch (std::exception &e)
        {
            return -1;
        }
        return 1;
    }
    const std::pair<int,int>& getIds()const{
        return _ids;
    }
protected:
    void writeMotors() override
    {
        auto goals = getMotorGoals();

        for (auto &goal : goals->motor_goals)
        {
            // no char type for messages, so grab first part of string
            if (goal.movement_type[0] == Motion::MovementType::VELOCITY)
            {
                _driver->set_velocity(DEFAULT_ADDRESS, std::make_pair(goal.motor_goal, goal.motor_goal)); // Needs tested, I believe this communicates a value to each motor
            }
            else
            {
                std::cerr << "Roboclaw motors only accept velocity commands.\n";
            }
        }
    }

    MotorStateList readMotors() override
    {
        MotorStateList list;
        MotorState state;
        _driver->get_velocity(DEFAULT_ADDRESS);
        return MotorStateList();
    }
private:
    std::shared_ptr<driver> _driver;
    std::pair<int,int> _ids;
    const char *_device_name;
    int _baudrate;
    float _protocol_version;
};

#endif