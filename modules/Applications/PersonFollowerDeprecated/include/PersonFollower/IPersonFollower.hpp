/* Â© Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include "CommonInterfaces/IInitializable.hpp"
#include "Types/TaskTypes/TaskVelocity.hpp"

namespace crf {
namespace applications {
namespace personfollower {

class IPersonFollower : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IPersonFollower() = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;
};

}  // namespace personfollower
}  // namespace applications
}  // namespace crf
