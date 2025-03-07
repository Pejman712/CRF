/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *         Jorge Playán Garai CERN BE/CEM/MRO 2021
 *  ==================================================================================================
 */

#include <memory>
#include <thread>
#include <chrono>

#include <nlohmann/json.hpp>

#include "DeviceManager/DeviceManagerWithAutoInitialization/DeviceManagerWithAutoInitialization.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace utility {
namespace devicemanager {

DeviceManagerWithAutoInitialization::DeviceManagerWithAutoInitialization(
    const std::shared_ptr<crf::utility::commoninterfaces::IInitializable>& device,
    const std::chrono::milliseconds& initializationTimeout) :
    logger_("DeviceManagerWithAutoInitialization"),
    accessMutex_(),
    device_(device),
    lastRequestTime_(),
    initializationTimeout_(initializationTimeout),
    deviceInitialized_(false),
    checkLatestRequestThread_(),
    requestTimeMutex_() {
    logger_->debug("CTor");
    if (device_ == nullptr) {
        throw std::runtime_error("Pointer to device is nullptr");
    }
}

DeviceManagerWithAutoInitialization::~DeviceManagerWithAutoInitialization() {
    logger_->debug("DTor");

    if (device_ != nullptr && deviceInitialized_) {
        std::scoped_lock<std::recursive_mutex> accessLock(accessMutex_);
        if (!device_->deinitialize()) {
            logger_->warn("Failed to deinitialize the device");
        }
    }
    deviceInitialized_ = false;
    initializationCV_.notify_one();
    if (checkLatestRequestThread_.joinable()) {
        checkLatestRequestThread_.join();
    }
}

nlohmann::json DeviceManagerWithAutoInitialization::getStatus() {
    logger_->debug("getStatus");
    std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
    nlohmann::json statusJSON;

    // The last request time is updated inside initializeDevice()
    if (initializeDevice()) {
        statusJSON["status"] = "initialize";
    } else {
        statusJSON["status"] = "deinitialize";
    }

    return statusJSON;
}

bool DeviceManagerWithAutoInitialization::initializeDevice() {
    refreshLastRequestTime();
    if (deviceInitialized_)
        return true;

    {
        std::scoped_lock<std::recursive_mutex> lock(accessMutex_);
        // Check again, it can happen that another thread was initializing the device
        // while this thread was waiting for the mutex.
        if (deviceInitialized_)
            return true;
        if (!(deviceInitialized_ = device_->initialize())) {
            logger_->error("Failed to initialize device");
            return false;
        }
    }
    if (initializationTimeout_ == std::chrono::milliseconds(0)) {
        return true;
    }
    if (checkLatestRequestThread_.joinable()) {
        checkLatestRequestThread_.join();
    }
    checkLatestRequestThread_ = std::thread(
        &DeviceManagerWithAutoInitialization::checkLatestRequestTime, this);
    return true;
}

void DeviceManagerWithAutoInitialization::refreshLastRequestTime() {
    std::scoped_lock<std::mutex> lock(requestTimeMutex_);
    lastRequestTime_ = std::chrono::high_resolution_clock::now();
}

void DeviceManagerWithAutoInitialization::checkLatestRequestTime() {
    while (deviceInitialized_) {
        auto currentTime = std::chrono::high_resolution_clock::now();

        std::unique_lock<std::mutex> requestTimeLock(requestTimeMutex_);
        auto timeSinceRequest = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime-lastRequestTime_);
        requestTimeLock.unlock();

        if (timeSinceRequest > initializationTimeout_) {
            std::scoped_lock<std::recursive_mutex> accessLock(accessMutex_);
            if (!device_->deinitialize()) {
                logger_->warn("Failed to deinitialize device - Keep trying");
            } else {
                deviceInitialized_ = false;
                logger_->info("Device deinitialized");
            }
        }

        /*
         * To avoid running this thread nonstop consuming resources we put a sleep ten times less
         * that the initializationTimeout_. Like this we ensure that if lastRequestTime_ does not
         * change there will be exactly 10 iterations of this loop before the call to deinitialize.
         * If during the sleep, lastRequestTime_ is updated we would de-synchronize and take longer
         * to deinitialize. That is why if timeSinceRequest_ is smaller than the standard sleep, we
         * instead sleep the time left for timeSinceRequest to reach timeinitializationTimeout_/10,
         * and in the following iterations we continue sleeping the standard time.
         * We use Conditional Variables to be able to stop the sleep when needed, for example in the
         * DTor
         */
        std::mutex initializationMtx;
        std::unique_lock<std::mutex> initializationLock(initializationMtx);
        if (timeSinceRequest >= initializationTimeout_/10) {
            initializationCV_.wait_for(initializationLock, initializationTimeout_/10);
        } else {
            initializationCV_.wait_for(initializationLock, (initializationTimeout_/10)-timeSinceRequest);  // NOLINT
        }
    }
}

}  // namespace devicemanager
}  // namespace utility
}  // namespace crf
