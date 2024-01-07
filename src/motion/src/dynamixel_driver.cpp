#include "motion/dynamixel_driver.hpp"
#include "motion/dynamixel_helper.hpp"
#include "telebot_interfaces/msg/motor_state.hpp"
#include <unordered_map>
#include <memory>
#include <functional>
#include "motion/dynamixel_addresses.hpp"
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

// void DynamixelDriver::updateMotors(std::function<void(MotorState&, int32_t)> updateCallback,std::unordered_map<int,MotorState> motorStates,DynamixelHelper& dh,std::vector<int64_t>& motorList){
//     for (auto &i : _xmMotors)
//     {
//         auto data = dh.getLastData(i);
//         auto val = data.first;
//         bool successful = data.second;

//         if(!successful){
//             continue;
//         }

//         auto found=motorStates.find(i);
//         if(found==motorStates.end()){
//             MotorState state;
//             state.id=i;
//             updateCallback(state,val);
//         }
//         else{
//             updateCallback(found->second,val);
//         }
       
//     }
//     for (auto &i : _proMotors)
//     {
//         auto data = dh.getLastData(i);
//         auto val = data.first;
//         bool successful = data.second;

//         if(!successful){
//             continue;
//         }

//         auto found=motorStates.find(i);
//         if(found==motorStates.end()){
//             MotorState state;
//             state.id=i;
//             updateCallback(state,val);
//             motorStates.insert(std::make_pair(i,state));
//         }
//         else{
//             updateCallback(found->second,val);
//         }
//     }
// }
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
    ReadWriteRule readPosition={RAM::PRESENT_POSITION_ADDR,RAM::PRESENT_POSITION_SIZE,setPosition};
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
    // std::cout<<"in read motors\n";
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
    std::cout<<msg.motor_states[0].position;
    std::cout<<msg.motor_states[1].position;
    return msg;
}