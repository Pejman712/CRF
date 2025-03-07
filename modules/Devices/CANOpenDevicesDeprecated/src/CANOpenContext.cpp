/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "CANOpenDevices/CANOpenContext.hpp"
#include "CANOpenDevices/CANOpenDefinitions.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

CANOpenContext::CANOpenContext(std::shared_ptr<communication::cansocket::ICANSocket> socket) :
    logger_("CANOpenContext-"+socket->getName()),
    socket_(socket),
    devices_(),
    initialized_(false),
    stopThreads_(false),
    receiverThread_(),
    sendSync_(false),
    syncFrequency_(0),
    sendGuard_(false),
    lastGuardSent_(),
    guardFrequency_(0),
    syncGuardSenderThread_() {
        logger_->debug("CTor");
}

CANOpenContext::~CANOpenContext() {
    logger_->debug("DTor");
    deinitialize();
}

bool CANOpenContext::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }
    if (!socket_->initialize()) {
        logger_->error("Initialization of CAN socket failed");
        return false;
    }

    stopThreads_ = false;
    receiverThread_ = std::thread(&CANOpenContext::receiver, this);

    if (sendSync_ || sendGuard_) {
        syncGuardSenderThread_ = std::thread(&CANOpenContext::syncGuardSender, this);
    }

    initialized_ = true;
    return true;
}

bool CANOpenContext::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    stopThreads_ = true;
    receiverThread_.join();

    if (sendSync_ || sendGuard_) {
        syncGuardSenderThread_.join();
    }
    if (!socket_->deinitialize()) {
        logger_->error("Denitialization of CAN socket failed");
        return false;
    }

    initialized_ = false;
    return true;
}

bool CANOpenContext::addDevice(std::shared_ptr<ICANOpenDevice> device) {
    logger_->debug("addMotor");
    if (devices_.find(device->getCANID()) != devices_.end()) {
        logger_->warn("A motor with the same ID has been already added: {}", device->getCANID());
        return false;
    }
    logger_->info("Added motor with ID {}", device->getCANID());
    devices_.insert({device->getCANID(), device});
    return true;
}

bool CANOpenContext::sendSync() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    can_frame syncFrame = {};
    syncFrame.can_id = 0x80;
    syncFrame.can_dlc = 0x00;

    socket_->write(&syncFrame);
    return true;
}

bool CANOpenContext::sendGuard() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    can_frame guardFrame = {};
    guardFrame.can_id = 0x700 + 0x25;
    guardFrame.can_dlc = 0x01;
    guardFrame.data[0] = NMTManagement::NMTOperational;

    socket_->write(&guardFrame);
    lastGuardSent_ = std::chrono::high_resolution_clock::now();
    return true;
}

bool CANOpenContext::setSyncFrequency(const std::chrono::milliseconds& frequency) {
    logger_->debug("setSyncFrequency");
    if (initialized_) {
        logger_->warn("CAN't set the sync frequency when already initialized");
        return false;
    }
    sendSync_ = true;
    syncFrequency_ = frequency;
    return true;
}

bool CANOpenContext::setGuardFrequency(const std::chrono::milliseconds& frequency) {
    logger_->debug("setGuardFrequency");
    if (initialized_) {
        logger_->warn("CAN't set the guard frequency when already initialized");
        return false;
    }
    sendGuard_ = true;
    guardFrequency_ = frequency;
    return true;
}

void CANOpenContext::receiver() {
    logger_->debug("receiver");
    can_frame frame;
    while (!stopThreads_) {
        int retval = socket_->read(&frame);
        if (retval <= 0) {
            logger_->warn("Failed reading from CAN");
            continue;
        }

        int motorID = frame.can_id & 0x007F;

        auto iterator = devices_.find(motorID);
        if (iterator == devices_.end()) {
            logger_->warn("Motor ID {} not found", motorID);
            continue;
        }

        iterator->second->getObjectDictionary()->setData(frame);
    }
}

void CANOpenContext::syncGuardSender() {
    logger_->debug("syncGuardSender");

    while (!stopThreads_) {
        if (sendGuard_) {
            auto now = std::chrono::high_resolution_clock::now();
            if (now - lastGuardSent_ > guardFrequency_) {
                sendGuard();
            }
        }

        if (sendSync_) sendSync();
        if (sendSync_) {
            std::this_thread::sleep_for(syncFrequency_);
        } else {
            std::this_thread::sleep_for(guardFrequency_);
        }
    }
}

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
