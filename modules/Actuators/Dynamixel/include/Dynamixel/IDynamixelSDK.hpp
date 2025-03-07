#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <dynamixel_sdk/dynamixel_sdk.h>

namespace crf {
namespace devices {
namespace dynamixelstepper {

class IDynamixelSDK {
 public:
    virtual ~IDynamixelSDK() = default;

    /**
     * [getPortHandler The function that gets PortHandler class inheritance]
     * @param port_name  ex: /dev/ttyUSB0
     * @return           [description]
     */
    virtual dynamixel::PortHandler *getPortHandler(const char *) = 0;
    /**
     * [getPacketHandler The function that returns PacketHandler instance]
     * @param protocol_version  1 or 2
     * @return PacketHandler    instance
     */
    virtual dynamixel::PacketHandler *getPacketHandler(float) = 0;

    /**
     * [openPort The function that opens the port]
     * @param  port PortHandler instance
     * @return communication results which come from PortHandlerLinux::setBaudRate()
     */
    virtual bool openPort(dynamixel::PortHandler *) = 0;

    /**
     * [closePort The function that closes the port]
     * @param  port   PortHandler instance
     */
    virtual void closePort(dynamixel::PortHandler *) = 0;

    /**
     * [setBaudRate The function that sets baudrate into the port handler]
     * @param  port      PortHandler instance
     * @param  baudrate
     * @return when error was occurred during port opening or true
     */
    virtual bool setBaudRate(dynamixel::PortHandler *, const int) = 0;

    /**
     * [write1ByteTxRx   The function calls PacketHandler::writeTxRx() for writing 1 byte data and
     receves the packet,gets the error from the packet.]
     * @param  packet  PacketHandler instance
     * @param  port    PortHandler instance
     * @param  id      Dynamixel ID
     * @param  address Address of the data for write
     * @param  data    Data for write
     * @param  error   Dynamixel hardware error
     * @return         communication results which come from PacketHandler::writeTxRx()
     */
    virtual int write1ByteTxRx(dynamixel::PacketHandler *, dynamixel::PortHandler *, uint8_t,
        uint16_t, uint8_t, uint8_t *) = 0;

    /**
     * [write2ByteTxRx   The function that calls PacketHandler::writeTxRx() for writing 2 byte data
     and receives the packet]
     * @param  packet  PacketHandler instance
     * @param  port    PortHandler instance
     * @param  id      Dynamixel ID
     * @param  address Address of the data for write
     * @param  data    Data for write
     * @param  error   Dynamixel hardware error
     * @return         communication results which come from PacketHandler::writeTxRx()
     */
    virtual int write2ByteTxRx(dynamixel::PacketHandler *, dynamixel::PortHandler *, uint8_t,
        uint16_t, uint16_t, uint8_t *) = 0;

    /**
     * [read2ByteTxRx    The function that calls PacketHandler::readTxRx() function for reading 2
     byte data]
     * @param  packet  PacketHandler instance
     * @param  port    PortHandler instance
     * @param  id      Dynamixel ID
     * @param  address Address of the data for read
     * @param  data    Data extracted from the packet
     * @param  error   Dynamixel hardware error
     * @return         communication results which come from PacketHandler::txRxPacket()
     */
    virtual int read2ByteTxRx(dynamixel::PacketHandler *, dynamixel::PortHandler *, uint8_t,
        uint16_t, uint16_t *, uint8_t *) = 0;

    /**
     * [getRxPacketError   The function that gets description of hardware error]
     * @param  packet PacketHandler instance
     * @param  error  Dynamixel hardware error which might be gotten by the tx rx functions
     * @return        description of hardware error in const char* (string)
     */
    virtual const char *getRxPacketError(dynamixel::PacketHandler *, uint8_t) = 0;
};

}  // namespace dynamixelstepper
}  // namespace devices
}  // namespace crf
