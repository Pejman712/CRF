/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO 2017
 *         Thomas Breant CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <string>
#include <thread>
#include <cerrno>
#include <termios.h>
#include <unistd.h>

#include "CANSocket/CANSocket.hpp"

namespace crf {
namespace communication {
namespace cansocket {

CANSocket::CANSocket(const std::string& interfaceName, int maskID) :
    interfaceName_(interfaceName),
    maskID_(maskID),
    address_(),
    ifr_(),
    fileDescriptor_(0),
    logger_("CANSocket") {
}

CANSocket::~CANSocket() {
    logger_->debug("DTor");
    deinitialize();
}

bool CANSocket::initialize() {
    if (fileDescriptor_ != 0) {
        logger_->warn("Socket already initilazed");
        return false;
    }
    if (open() == -1) {
        return false;
    }
    strcpy(ifr_.ifr_name, interfaceName_.c_str());  // NOLINT
    ioctl(fileDescriptor_, SIOCGIFINDEX, &ifr_);

    address_.can_family = AF_CAN;
    address_.can_ifindex = ifr_.ifr_ifindex;

    // Sets a timout for the read call to 2 seconds
    struct timeval tv;
    tv.tv_sec = readTimeOutSeconds_;
    tv.tv_usec = 0;

    // Helps in reuse of address and port for the socket fileDescriptor_ (optionnal)
    if (::setsockopt(fileDescriptor_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv) == -1) {
        logger_->warn("Failed to set socket option 1 : {}", std::strerror(errno));
        return false;
    }

    if (maskID_ != -1) {
        struct can_filter rfilter;
        rfilter.can_id = maskID_;
        rfilter.can_mask = 0x00F;
        if (::setsockopt(fileDescriptor_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) == -1) {  // NOLINT
            logger_->warn("Failed to set socket option 2 : {}", std::strerror(errno));
            return false;
        }
    }

    // Binds the socket to the address and port number specified
    if (::bind(fileDescriptor_, (struct sockaddr *)&address_, sizeof(address_)) == -1) {
        logger_->error("Error in socket bind : {}", std::strerror(errno));
        return false;
    }

    logger_->debug("Socket correctly bind");
    return true;
}

bool CANSocket::deinitialize() {
    logger_->info("deinitialize");
    if (fileDescriptor_ == 0) {
        logger_->warn("Not initialized");
        return false;
    }
    if (::close(fileDescriptor_) == -1) {
        logger_->warn("Failed to close: {}", std::strerror(errno));
        return false;
    }
    fileDescriptor_ = 0;
    return true;
}

int CANSocket::open() {
    /*
     * The inputs are Linux defined:
     *  - PF_CAN: new protocol family including a CAN_RAW protocol <=> communication domain.
     *  - SOCK_RAW: communication type.
     *  - CAN_RAW: protocol (IP) supporting multiple CAN frame transmission.
     */
    if ((fileDescriptor_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        logger_->error("Error while opening socket");
        return -1;
    } else {
        logger_->debug("Socket opened: {}", fileDescriptor_);
        return fileDescriptor_;
    }
}

int CANSocket::write(can_frame* frame) {
    if (fileDescriptor_ <= 0) {
        logger_->warn("Cannot write because not initialized");
        return -1;
    }
    int numBytesSent = ::write(fileDescriptor_, frame, sizeof(can_frame));
    if (numBytesSent == -1) {
        logger_->warn("Failed to write: {}", std::strerror(errno));
    }
    return numBytesSent;
}

int CANSocket::read(can_frame* frame) {
    if (fileDescriptor_ <= 0) {
        logger_->warn("Cannot read because not initialized");
        return -1;
    }
    int numBytesRead =::read(fileDescriptor_, frame, sizeof(struct can_frame));
    if (numBytesRead == -1) {
        logger_->warn("Failed to read : {}", std::strerror(errno));
    } else if (static_cast<unsigned int>(numBytesRead) < sizeof(can_frame)) {
        logger_->warn("Failed to read, frame not full : {}", std::strerror(errno));
    }
    return numBytesRead;
}

std::string CANSocket::getName() const {
    return interfaceName_;
}

void CANSocket::printFrame(can_frame frame) {
    logger_->debug("id: {}, dlc: {}, data: {} {} {} {} {} {} {} {}",
        frame.can_id, frame.can_dlc, frame.data[0], frame.data[1], frame.data[2],
        frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
}

void CANSocket::printFrame(can_frame* frame) {
    logger_->debug("id: {}, dlc: {}, data: {} {} {} {} {} {} {} {}",
        frame->can_id, frame->can_dlc, frame->data[0], frame->data[1], frame->data[2],
        frame->data[3], frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
}

}  // namespace cansocket
}  // namespace communication
}  // namespace crf
