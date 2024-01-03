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

#include "motion/driver.hpp"
#include "motion/motion.hpp"

#include "roboclaw_sdk/roboclaw_driver.h"

#include <unordered_map>
#include <vector>

class RoboclawDriver : public MotorDriver {
public:
    RoboclawDriver(): MotorDriver() {}

    RoboclawDriver(int baudrate, std::vector<int> motorIDs) : MotorDriver(), _port(port),
                                                                           _baudrate(baudrate), _motorIDs(motorIDs) {}

    int initialize() {
        std::for_each(_motorIDs.cbegin(), _motorIDs.cend(), [_drivers, _baudrate](int id){
            //TODO: figure out the ports..?
            driver temp = new driver("tempPort", _baudrate);
            // This might not be the best idea, not sure how it will play out...
            temp.reset_encoders();
            _drivers[id] = temp;
        })
    }

private:
    std::unordered_map<int, driver> _drivers;

    std::vector<int> _motorIDs;
    int _baudrate;

}

#endif