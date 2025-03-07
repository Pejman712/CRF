/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#pragma once

#include "ThermalCamera/IOptrisDeviceFactory.hpp"

#include <memory>
#include <string>

#include <libirimager/IRDeviceUVC.h>
#include <libirimager/IRDeviceParams.h>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace thermalcamera {

class OptrisDeviceFactory: public IOptrisDeviceFactory {
 public:
    OptrisDeviceFactory() = delete;
    explicit OptrisDeviceFactory(const std::string& configFile);
    ~OptrisDeviceFactory() override = default;
    std::shared_ptr<evo::IRDevice> createDevice() override;
    std::shared_ptr<evo::IRImager> createImager(unsigned int frequency,
        unsigned int width, unsigned int height) override;

 private:
    utility::logger::EventLogger logger_;
    evo::IRDeviceParams params_;
};

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
