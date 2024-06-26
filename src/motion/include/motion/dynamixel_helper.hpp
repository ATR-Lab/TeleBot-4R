#ifndef DYNAMIXEL_HELPER_HPP
#define DYNAMIXEL_HELPER_HPP
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <memory>
using dynamixel::GroupSyncRead;
class DynamixelHelper{
public:
    DynamixelHelper(dynamixel::PortHandler* portHandler,dynamixel::PacketHandler* packetHandler){
        _packetHandler=packetHandler;
        _portHandler=portHandler;
    }

    /// @brief Gets the last data read by one of the synchronous read functions. Eg. readPositions()
    /// @param id Motor id to retrieve value
    /// @return Pair of the value read and if it was available.
    std::pair<uint32_t,bool> getLastData(uint8_t id){
        return std::make_pair(_syncReader->getData(id,_currentAddr,_currentLen),
        _syncReader->isAvailable(id,_currentAddr,_currentLen));
    }
    void setReader(uint16_t,uint16_t);
    void executeRead(){
        _syncReader->txRxPacket();
    }
    void addParams(const std::vector<int64_t>&);
private:
    uint16_t _currentAddr,_currentLen;
    dynamixel::PacketHandler* _packetHandler;
    dynamixel::PortHandler* _portHandler;
    std::unique_ptr<GroupSyncRead> _syncReader;
};
#endif