#include "motion/dynamixel_helper.hpp"
#include "motion/dynamixel_addresses.hpp"
using dynamixel::GroupSyncRead;
using namespace DynamixelAddresses;

void DynamixelHelper::setReader(uint16_t address,uint16_t size){
   
    _currentAddr=address;
    _currentLen=size;

    _syncReader=std::make_unique<GroupSyncRead>(GroupSyncRead(_portHandler,_packetHandler,_currentAddr,_currentLen));
}
void DynamixelHelper::addParams(const std::vector<int64_t>& ids){
    for(auto&id:ids){
        _syncReader->addParam(id);
    }
}