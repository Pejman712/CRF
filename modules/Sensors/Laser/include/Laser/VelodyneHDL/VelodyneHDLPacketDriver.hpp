/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO
 *
 * Original code
 * Velodyne HDL Packet Driver
 * Nick Rypkema (rypkema@mit.edu), MIT 2017
 * Shared library to read a Velodyne HDL packet streaming over UDP
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <boost/asio.hpp>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace laser {

class VelodyneHDLPacketDriver {
 public:
    VelodyneHDLPacketDriver();
    explicit VelodyneHDLPacketDriver(unsigned int port);
    virtual ~VelodyneHDLPacketDriver();
    bool InitPacketDriver(unsigned int port);
    bool GetPacket(std::string* data, unsigned int* dataLength);

 protected:
    void GetPacketCallback(const boost::system::error_code& error, std::size_t numBytes,
        std::string* data, unsigned int* dataLength);

 private:
    crf::utility::logger::EventLogger logger_;
    unsigned int port_;
    char rxBuffer_[1500];
    boost::asio::io_service ioService_;
    boost::shared_ptr<boost::asio::ip::udp::socket> socket_;
};

}  // namespace laser
}  // namespace sensors
}  // namespace crf
