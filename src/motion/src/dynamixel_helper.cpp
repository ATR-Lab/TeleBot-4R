#include "motion/dynamixel_helper.hpp"
#include "motion/dynamixel_addresses.hpp"
using dynamixel::GroupSyncRead;
using namespace DynamixelAddresses;

void DynamixelHelper::setReader(std::pair<uint16_t,uint16_t> xm,std::pair<uint16_t,uint16_t> pro, bool isPro){
    if(isPro){
        _currentAddr=xm.first;
        _currentLen=xm.second;
    }
    else{
        _currentAddr=pro.first;
        _currentLen=pro.second;
    }
    _syncReader=std::make_unique<GroupSyncRead>(GroupSyncRead(_portHandler,_packetHandler,_currentAddr,_currentLen));
}
void DynamixelHelper::addParams(const std::vector<int64_t>& ids){
    for(auto&id:ids){
        _syncReader->addParam(id);
    }
}
void DynamixelHelper::readPositions(const std::vector<int64_t>& ids,bool isPro){
    //Prime reader for addition of motors to tx.
    setReader(std::make_pair(XM540::RAM::PRESENT_POSITION_ADDR,XM540::RAM::PRESENT_POSITION_SIZE),
    std::make_pair(Pro::RAM::PRESENT_POSITION_ADDR,Pro::RAM::PRESENT_POSITION_SIZE),isPro);
    addParams(ids);
    _syncReader->txRxPacket();
}

void DynamixelHelper::readErrors(const std::vector<int64_t>& ids,bool isPro){
    setReader(std::make_pair(XM540::RAM::HARDWARE_ERROR_STATUS_ADDR,XM540::RAM::HARDWARE_ERROR_STATUS_SIZE),
    std::make_pair(Pro::RAM::HARDWARE_ERROR_STATUS_ADDR,Pro::RAM::HARDWARE_ERROR_STATUS_SIZE),isPro);
    addParams(ids);
    
}

void DynamixelHelper::readMoving(const std::vector<int64_t>& ids,bool isPro){
    setReader(std::make_pair(XM540::RAM::MOVING_ADDR,XM540::RAM::MOVING_SIZE),
    std::make_pair(Pro::RAM::MOVING_ADDR,Pro::RAM::MOVING_SIZE),isPro);
    addParams(ids);

}

void DynamixelHelper::readCurrent(const std::vector<int64_t>& ids,bool isPro){
    setReader(std::make_pair(XM540::RAM::PRESENT_CURRENT_ADDR,XM540::RAM::PRESENT_CURRENT_SIZE),
    std::make_pair(Pro::RAM::PRESENT_CURRENT_ADDR,Pro::RAM::PRESENT_CURRENT_SIZE),isPro);
    addParams(ids);

}

void DynamixelHelper::readVelocity(const std::vector<int64_t>& ids,bool isPro){
    setReader(std::make_pair(XM540::RAM::PRESENT_VELOCITY_ADDR,XM540::RAM::PRESENT_VELOCITY_SIZE),
    std::make_pair(Pro::RAM::PRESENT_VELOCITY_ADDR,Pro::RAM::PRESENT_VELOCITY_SIZE),isPro);
    addParams(ids);

}