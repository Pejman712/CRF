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

#include <iostream>
#include <string>
#include <boost/bind.hpp>

#include "Laser/VelodyneHDL/VelodyneHDLPacketDriver.hpp"

namespace crf {
namespace sensors {
namespace laser {

VelodyneHDLPacketDriver::VelodyneHDLPacketDriver():
    logger_("VelodyneHDLPacketDriver") {
    logger_->debug("CTor");
}

VelodyneHDLPacketDriver::VelodyneHDLPacketDriver(unsigned int port):
    port_(port),
    logger_("VelodyneHDLPacketDriver") {
    logger_->debug("CTor");
    if (!InitPacketDriver(port_)) {
        throw std::runtime_error("Failed to bind Velodyne HDL socket");
    }
}

VelodyneHDLPacketDriver::~VelodyneHDLPacketDriver() {
    logger_->debug("DTor");
}

bool VelodyneHDLPacketDriver::InitPacketDriver(unsigned int port) {
    port_ = port;
    boost::asio::ip::udp::endpoint destination_endpoint(boost::asio::ip::address_v4::any(), port_);
    try {
        socket_ = boost::shared_ptr<boost::asio::ip::udp::socket>(
            new boost::asio::ip::udp::socket(ioService_));
        socket_->open(destination_endpoint.protocol());
        socket_->bind(destination_endpoint);
    } catch(std::exception & e) {
        logger_->warn("Error binding to socket - {}. Trying once more...", e.what());
        try {
            destination_endpoint = boost::asio::ip::udp::endpoint(
                boost::asio::ip::address_v4::any(), port_);
            socket_ = boost::shared_ptr<boost::asio::ip::udp::socket>(
                new boost::asio::ip::udp::socket(ioService_));
            socket_->open(destination_endpoint.protocol());
            socket_->bind(destination_endpoint);
        } catch(std::exception & e) {
            logger_->error("Error binding to socket - {}. Failed", e.what());
            return false;
        }
    }
    logger_->info("Success binding to Velodyne HDL socket");
    return true;
}

bool VelodyneHDLPacketDriver::GetPacket(std::string* data, unsigned int* dataLength) {
    try {
        socket_->async_receive(boost::asio::buffer(rxBuffer_, 1500), boost::bind(
            &VelodyneHDLPacketDriver::GetPacketCallback, this, boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred, data, dataLength));
        ioService_.reset();

        // Maximun allowed time (version 1.66 of boost)
        // std::chrono::milliseconds maxTime{1000};
        // if (ioService_.run_for(maxTime) == 0) {
        //     return false;
        // }

        ioService_.run();
        return true;
    } catch(std::exception & e) {
        logger_->error("Error receiving packet {}", e.what());
        return false;
    }
}

void VelodyneHDLPacketDriver::GetPacketCallback(const boost::system::error_code& error,
    std::size_t numBytes, std::string* data, unsigned int* dataLength) {

    // if (rxBuffer_[1205] == '\0')
    //     throw std::runtime_error("Buffer size is not 1205");

    (*data).assign(rxBuffer_, numBytes);
    *dataLength = (unsigned int) numBytes;
    return;
}

}  // namespace laser
}  // namespace sensors
}  // namespace crf
