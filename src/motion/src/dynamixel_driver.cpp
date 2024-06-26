#include "motion/dynamixel_driver.hpp"
#include "motion/dynamixel_helper.hpp"
#include "telebot_interfaces/msg/motor_state.hpp"
#include <unordered_map>
#include <memory>
#include <functional>
#include "motion/dynamixel_addresses.hpp"
using telebot_interfaces::msg::MotorState;

//Callbacks for the rules
void setPosition(MotorState& state,uint32_t val){
    state.position=DynamixelDriver::ticksToRadians(val,false);
}
void setPositionPro(MotorState& state,uint32_t val){
    state.position=DynamixelDriver::ticksToRadians(val,true);
}
void setVelocity(MotorState& state,uint32_t val){
    state.speed=val;
}
void setError(MotorState& state,uint32_t val){
    state.error=val;
}
/// @brief IMPORTANT: This takes a uint32_t so that the callback can work with the class, it should be noted that the 2 byte size of the dynamixe
/// @param state 
/// @param val 
void setLoad(MotorState& state,uint32_t val){
    //This takes the low bytes and reinterprets them as 2's complement signed int16
    auto signedConversion=(int16_t)val;
    //This converts it to a percentage.
    auto floatConversion=signedConversion/10.0; /*
    We divide by ten because the dynamixel motors represent a percentage as the signed int value divided by 10. So for example, 1 is equal to 0.1% and -100 is -10%.
    Negative means its pushing clockwise, positive means counterclockwise. AKA positive is a force going clockwise and vice versa.
    */
    state.load=floatConversion; //This is supposed to be a c
}
void setMoving(MotorState& state,uint32_t val){
    state.moving=val;
}

const std::vector<ReadWriteRule> getXMRules(){
    using namespace DynamixelAddresses::XM540;
    ReadWriteRule readPosition={RAM::PRESENT_POSITION_ADDR,RAM::PRESENT_POSITION_SIZE,setPosition};
    ReadWriteRule readErrors={RAM::HARDWARE_ERROR_STATUS_ADDR,RAM::HARDWARE_ERROR_STATUS_SIZE,setError};
    ReadWriteRule readVelocity={RAM::PRESENT_VELOCITY_ADDR,RAM::PRESENT_VELOCITY_SIZE,setVelocity};
    ReadWriteRule readMoving={RAM::MOVING_ADDR,RAM::MOVING_SIZE,setMoving};
    ReadWriteRule readLoad={RAM::PRESENT_CURRENT_ADDR,RAM::PRESENT_CURRENT_SIZE,setLoad};
    const std::vector<ReadWriteRule> rules={{readPosition,readErrors,readVelocity,readMoving,readLoad}};
    return rules;
}
const std::vector<ReadWriteRule> getProRules(){
    using namespace DynamixelAddresses::Pro;
    ReadWriteRule readPosition={RAM::PRESENT_POSITION_ADDR,RAM::PRESENT_POSITION_SIZE,setPositionPro};
    ReadWriteRule readErrors={RAM::HARDWARE_ERROR_STATUS_ADDR,RAM::HARDWARE_ERROR_STATUS_SIZE,setError};
    ReadWriteRule readVelocity={RAM::PRESENT_VELOCITY_ADDR,RAM::PRESENT_VELOCITY_SIZE,setVelocity};
    ReadWriteRule readMoving={RAM::MOVING_ADDR,RAM::MOVING_SIZE,setMoving};
    ReadWriteRule readLoad={RAM::PRESENT_CURRENT_ADDR,RAM::PRESENT_CURRENT_SIZE,setLoad};
    const std::vector<ReadWriteRule> rules={{readPosition,readErrors,readVelocity,readMoving,readLoad}};
    return rules;
}

void DynamixelDriver::runRules(std::vector<int64_t>& ids,const std::vector<ReadWriteRule>& rules,MotorState motorStates[]){
    for(const auto& rule:rules){
        // Set sync reader to read from this rule
        _dh->setReader(rule.address,rule.size);
        //Populate motor ids in reader
        _dh->addParams(ids);
        // Read from motors
        _dh->executeRead();
        // Populate motor list
        for(auto&id:ids){
            auto res=_dh->getLastData(id);
            auto value=res.first;
            bool successful=res.second;
            //Check read success
            if(successful==true){
                //Update motor at
                rule.callback(motorStates[id],value);
                //Set success marker if not set
                successfulReads[id]=true;
            }
        }
    }
}
MotorStateList DynamixelDriver::readMotors()
{
    //Get our rules one time
    static const std::vector<ReadWriteRule> xmRules=getXMRules();
    static const std::vector<ReadWriteRule> proRules=getProRules();

    //Prime for reading
    MotorState motorStates[READ_ARRAY_SZ];
    resetSuccessArray();

    //Read motors and populate array
    runRules(_xmMotors,xmRules,motorStates);
    runRules(_proMotors,proRules,motorStates);

    //Convert to message
    MotorStateList msg;
    //Put motors with successful messages into the array
    for(int i=0;i<READ_ARRAY_SZ;i++){
        if(successfulReads[i]==true){
            motorStates[i].id=i;
            msg.motor_states.push_back(motorStates[i]);
        }
    }
    return msg;
}