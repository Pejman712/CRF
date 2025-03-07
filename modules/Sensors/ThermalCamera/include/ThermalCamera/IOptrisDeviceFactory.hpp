/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#pragma once

#include <memory>

// FWDs
namespace evo {
class IRDevice;
class IRImager;
}  // namespace evo

namespace crf {
namespace sensors {
namespace thermalcamera {

/**
* Interface class for Optris-specific classes factory
* Known subclasses: OptrisDeviceFactory, OptrisDeviceFactoryMock
*/
class IOptrisDeviceFactory {
 public:
    IOptrisDeviceFactory() = default;
    virtual ~IOptrisDeviceFactory() = default;
    virtual std::shared_ptr<evo::IRDevice> createDevice() = 0;
    virtual std::shared_ptr<evo::IRImager> createImager(unsigned int frequency,
        unsigned int width, unsigned int height) = 0;
};

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
