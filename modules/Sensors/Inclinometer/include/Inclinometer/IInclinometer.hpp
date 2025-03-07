#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>

#include "CommonInterfaces/IInitializable.hpp"

namespace crf {
namespace sensors {
namespace inclinometer {

class IInclinometer: public utility::commoninterfaces::IInitializable {
 public:
    IInclinometer() = default;
    IInclinometer(const IInclinometer& other) = delete;
    IInclinometer(IInclinometer&& other) = delete;
    ~IInclinometer() override = default;
    virtual std::vector<double> getInclination() = 0;
};

}  // namespace inclinometer
}  // namespace sensors
}  // namespace crf
