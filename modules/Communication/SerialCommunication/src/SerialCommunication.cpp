/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO 2018
 *
 *  ==================================================================================================
 */

#include <fcntl.h>
#include <termios.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <unistd.h>
#include <thread>

#include "EventLogger/EventLogger.hpp"
#include "SerialCommunication/SerialCommunication.hpp"

namespace crf {
namespace communication {
namespace serialcommunication {

SerialCommunication::SerialCommunication(const std::string& deviceName, int baudRate,
    bool blocking, bool parityBit, int charSize,
    std::chrono::duration<float> timeout):
    logger_("SerialCommunication"),
    deviceName_(deviceName),
    baudRate_(baudRate),
    blocking_(blocking),
    parityBit_(parityBit),
    charSize_(charSize),
    timeout_(timeout),
    serialFd_(0) {
        logger_->debug("CTor");
}

bool SerialCommunication::initialize() {
    logger_->debug("initialize");
    if (serialFd_ != 0) {
        logger_->warn("Already initialized");
        return false;
    }
    int fileFlags = O_RDWR | O_NOCTTY | O_NDELAY;
    if (!blocking_) {
        fileFlags |= O_NONBLOCK;
    }
    serialFd_ = open(deviceName_.c_str(), fileFlags);
    if (serialFd_ == -1) {
        logger_->error("Error occured on attempt to open {}: {}", deviceName_,
            std::strerror(errno));
        serialFd_ = 0;
        return false;
    }
    struct termios portSettings{};
    switch (baudRate_) {
        case 9600:
            portSettings.c_cflag = B9600;
            break;
        case 19200:
            portSettings.c_cflag = B19200;
            break;
        case 38400:
            portSettings.c_cflag = B38400;
            break;
        case 57600:
            portSettings.c_cflag = B57600;
            break;
        case 115200:
            portSettings.c_cflag = B115200;
            break;
        default:
            logger_->warn("Unsupported baudrate: {}", baudRate_);
            return false;
    }
    switch (charSize_) {
        case 8:
            portSettings.c_cflag |= CS8;
            break;
        case 7:
            portSettings.c_cflag |= CS7;
            break;
        case 6:
            portSettings.c_cflag |= CS6;
            break;
        case 5:
            portSettings.c_cflag |= CS5;
            break;
        default:
            logger_->warn("Unsupported char size: {}", charSize_);
            return false;
    }
    portSettings.c_cflag |= CREAD | CLOCAL;
    portSettings.c_iflag = IGNPAR;
    if (parityBit_) {
        portSettings.c_cflag |= PARENB;
    }
    portSettings.c_oflag = 0;
    portSettings.c_lflag = 0;
    portSettings.c_cc[VMIN] = blocking_ ? 1 : 0;
    portSettings.c_cc[VTIME] = 5;
    tcsetattr(serialFd_, TCSANOW, &portSettings);
    tcflush(serialFd_, TCIFLUSH);
    return true;
}

bool SerialCommunication::deinitialize() {
    logger_->debug("deinitialize");
    if (serialFd_ == 0) {
        return true;
    }
    close(serialFd_);
    serialFd_ = 0;
    return true;
}

SerialCommunication::~SerialCommunication() {
    logger_->debug("DTor");
    deinitialize();
}

int SerialCommunication::read(std::string* buff, int length) {
    logger_->debug("read: {}", length);
    if (serialFd_ <= 0) {
        logger_->warn("Cannot read because not initialized");
        return -1;
    }
    std::unique_ptr<char[]> tmp(new char[length]);
    int numBytesRead = 0;
    if (blocking_) {
        auto start = std::chrono::high_resolution_clock::now();
        do {
            int retval = ::read(serialFd_, tmp.get() + numBytesRead, length - numBytesRead);
            if (retval > -1) {
                numBytesRead += retval;
            }
            if (retval <= -1 && errno != EAGAIN) {
                logger_->warn("Failed to read: {}", std::strerror(errno));
                return numBytesRead;
            }
            int timeToWait = 1e6*(length-numBytesRead)/(baudRate_*charSize_/8);
            std::this_thread::sleep_for(std::chrono::microseconds(timeToWait));

            if ((timeout_.count() != 0) && (std::chrono::high_resolution_clock::now() - start > timeout_)) {  // NOLINT
                logger_->warn("Read timeout");
                return numBytesRead;
            }
        } while (numBytesRead < length);
    } else {
        numBytesRead = ::read(serialFd_, tmp.get(), length);
    }
    if (numBytesRead > 0) {
        buff->assign(tmp.get(), numBytesRead);
    }
    if (numBytesRead == -1) {
        logger_->warn("Failed to read: {}", std::strerror(errno));
    }
    logger_->debug("Number of bytes read: {}", numBytesRead);
    return numBytesRead;
}

int SerialCommunication::write(const std::string& buff) {
    logger_->debug("write: {}", buff.size());
    if (serialFd_ <= 0) {
        logger_->warn("Cannot write because not initialized");
        return -1;
    }
    int numBytesSent = ::write(serialFd_, buff.c_str(), buff.size());
    if (numBytesSent == -1) {
        logger_->warn("Failed to write: {}", std::strerror(errno));
    }
    logger_->debug("Number of bytes sent: {}", numBytesSent);
    return numBytesSent;
}

}  // namespace serialcommunication
}  // namespace communication
}  // namespace crf
