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

#include "motion/driver.hpp"
#include "motion/motion.hpp"

#include "roboclaw_sdk/driver.h"

#include <unordered_map>
#include <vector>
#include <string>

class RoboclawDriver : public MotorDriver {
public:
    RoboclawDriver(): MotorDriver() {}

    RoboclawDriver(int baudrate, std::unordered_map<int, std::string> motorPorts) : MotorDriver(), _baudrate(baudrate), _motorPorts(motorPorts) {}

    int initialize() override {
        std::for_each(_motorPorts.cbegin(), _motorPorts.cend(), [this](std::pair<int, std::string> val){
            driver temp(val.second, _baudrate);
            int id = val.first;
            // This might not be the best idea, not sure how it will play out...
            temp.reset_encoders(DEFAULT_ADDRESS);
            // Creates the pair and inserts it
            _drivers.emplace(id, temp);
        });
    }

protected:
    void writeMotors() override {
        auto goals = getMotorGoals();

        for(auto &goal : goals->motor_goals) {
            // NOTE: the speed parameter of this set_velocty might be wrong. It mentions that it should be big endian.
            _drivers[goal.motor_id].set_velocity(DEFAULT_ADDRESS, std::make_pair(goal.motor_goal, goal.motor_goal));
        }
    }

    MotorStateList readMotors() override {
        return MotorStateList();
    }

private:
    std::unordered_map<int, driver> _drivers;

    std::unordered_map<int, std::string> _motorPorts;
    int _baudrate;
};

#endif