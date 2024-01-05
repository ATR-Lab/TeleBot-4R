#include "motion/dynamixel_driver.hpp"
#include "motion/dynamixel_helper.hpp"
#include "telebot_interfaces/msg/motor_state.hpp"
#include <unordered_map>
#include <memory>
#include <functional>
using telebot_interfaces::msg::MotorState;


void setPosition(MotorState& state,int32_t val){
    state.position=val;
}
void setVelocity(MotorState& state,int32_t val){
    state.speed=val;
}
void setError(MotorState& state,int32_t val){
    state.error=val;
}
void setLoad(MotorState& state,int32_t val){
    state.load=val;
}
void setMoving(MotorState& state,int32_t val){
    state.moving=val;
}

void DynamixelDriver::updateMotors(std::function<void(MotorState&, int32_t)> updateCallback,std::unordered_map<int,MotorState> motorStates,DynamixelHelper& dh){
    for (auto &i : _xmMotors)
    {
        auto data = dh.getLastData(i);
        auto val = data.first;
        bool successful = data.second;

        if(!successful){
            continue;
        }

        auto found=motorStates.find(i);
        if(found==motorStates.end()){
            MotorState state;
            state.id=i;
            updateCallback(state,val);
        }
        else{
            updateCallback(found->second,val);
        }
       
    }
    for (auto &i : _proMotors)
    {
        auto data = dh.getLastData(i);
        auto val = data.first;
        bool successful = data.second;

        if(!successful){
            continue;
        }

        auto found=motorStates.find(i);
        if(found==motorStates.end()){
            MotorState state;
            state.id=i;
            updateCallback(state,val);
            motorStates.insert(std::make_pair(i,state));
        }
        else{
            updateCallback(found->second,val);
        }
    }
}
MotorStateList DynamixelDriver::readMotors()
{
    
    static DynamixelHelper dh(_portHandler, _packetHandler);
    std::unordered_map<int,MotorState> motorStates;
    MotorStateList msg;
    dh.readPositions(_xmMotors, false);
    dh.readPositions(_proMotors, true);
    updateMotors(&setPosition,motorStates,dh);
    // MotorState state;
    
    for (auto id : _proMotors)
    {
    }

    // Get data from motors
    for (auto id : _proMotors)
    {
    }
    // primeMotorStates
    // addParams sync reader
    // Load values into state
    // auto syncReader=GroupSyncRead(_portHandler,_packetHandler,DynamixelAddresses::XM540::RAM::POSITIon)

    return msg;
}