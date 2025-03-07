#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <utility>

#include "CommonInterfaces/IInitializable.hpp"

namespace crf {
namespace devices {
namespace pantilt {

class IPanTilt : public utility::commoninterfaces::IInitializable {
 public:
    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * [DynamixelPanTilt::getPosition description]
     * @return The position in radians of each motor of the chain
     */
    virtual std::pair<float, float> getPosition() = 0;

    /**
     * [DynamixelPanTilt::setPosition description]
     * @param  coordinates the goal position for each motor in radians
     * @return true if everything was right
     */
    virtual bool setPosition(const std::pair<float, float> &) = 0;
};

}  // namespace pantilt
}  // namespace devices
}  // namespace crf
