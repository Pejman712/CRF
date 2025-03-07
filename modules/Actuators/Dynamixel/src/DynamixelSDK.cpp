/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <Dynamixel/DynamixelSDK.hpp>

namespace crf {
namespace devices {
namespace dynamixelstepper {

dynamixel::PortHandler *DynamixelSDK::getPortHandler(const char *port_name) {
    return dynamixel::PortHandler::getPortHandler(port_name);
}

dynamixel::PacketHandler *DynamixelSDK::getPacketHandler(float protocol_version) {
    return dynamixel::PacketHandler::getPacketHandler(protocol_version);
}

bool DynamixelSDK::openPort(dynamixel::PortHandler *portHandler) {
    return portHandler->openPort();
}

void DynamixelSDK::closePort(dynamixel::PortHandler *portHandler) {
    portHandler->closePort();
}

bool DynamixelSDK::setBaudRate(dynamixel::PortHandler *portHandler, const int baudrate) {
    return portHandler->setBaudRate(baudrate);
}

int DynamixelSDK::write1ByteTxRx(dynamixel::PacketHandler *packetHandler,
    dynamixel::PortHandler *portHandler, uint8_t id, uint16_t address, uint8_t data,
    uint8_t *error = 0) {
    return packetHandler->write1ByteTxRx(portHandler, id, address, data, error);
}

int DynamixelSDK::write2ByteTxRx(dynamixel::PacketHandler *packetHandler,
    dynamixel::PortHandler *portHandler, uint8_t id, uint16_t address, uint16_t data,
    uint8_t *error = 0) {
    return packetHandler->write2ByteTxRx(portHandler, id, address, data, error);
}

int DynamixelSDK::read2ByteTxRx(dynamixel::PacketHandler *packetHandler,
    dynamixel::PortHandler *portHandler, uint8_t id, uint16_t address, uint16_t *data,
    uint8_t *error = 0) {
    return packetHandler->read2ByteTxRx(portHandler, id, address, data, error);
}

const char *DynamixelSDK::getRxPacketError(dynamixel::PacketHandler *packetHandler, uint8_t error) {
    return packetHandler->getRxPacketError(error);
}

}  // namespace dynamixelstepper
}  // namespace devices
}  // namespace crf
