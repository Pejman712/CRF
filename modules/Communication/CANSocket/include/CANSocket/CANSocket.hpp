/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO 2017
 *         Thomas Breant CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <string>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "CANSocket/ICANSocket.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace communication {
namespace cansocket {

/*
 * @brief Implementation of the Linux CAN functions
 */
class CANSocket : public ICANSocket {
 public:
    explicit CANSocket(const std::string& interfaceName, int maskID = -1);
    CANSocket(const CANSocket& other) = delete;
    CANSocket(CANSocket&& other) = delete;
    CANSocket() = delete;
    ~CANSocket() override;

    bool initialize() override;
    bool deinitialize() override;

    int write(can_frame* frame) override;
    int read(can_frame* frame) override;
    std::string getName() const override;

    /*
     * @brief Prints the given frame in the right format.
     * @param Linux CAN frame structure that you want to print.
     */
    void printFrame(can_frame frame);
    void printFrame(can_frame* frame);

 private:
    std::string interfaceName_;
    int maskID_;
    struct sockaddr_can address_;
    struct ifreq ifr_;
    int fileDescriptor_;
    crf::utility::logger::EventLogger logger_;

    const time_t readTimeOutSeconds_ = 2;

    int open();
};

}  // namespace cansocket
}  // namespace communication
}  // namespace crf
