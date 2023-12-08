#include "motion/dynamixel_driver.hpp"
#include "motion/dynamixel_helper.hpp"
#include "telebot_interfaces/msg/motor_state.hpp"
using telebot_interfaces::msg::MotorState;
MotorStateList DynamixelDriver::readMotors()
{        
        static DynamixelHelper dh(_portHandler,_packetHandler);
        MotorStateList msg;
        MotorState state;
        dh.readLoads();
        for(auto id:_xmMotors){
            
        }

        //Get data from motors
        for(auto id:_proMotors){

        }
        // primeMotorStates
        // addParams sync reader
        // Load values into state
        // auto syncReader=GroupSyncRead(_portHandler,_packetHandler,DynamixelAddresses::XM540::RAM::POSITIon)

        return list;
}