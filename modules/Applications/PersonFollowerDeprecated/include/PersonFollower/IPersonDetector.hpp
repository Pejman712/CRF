#pragma once

/* Â© Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <opencv2/core.hpp>

#include "CommonInterfaces/IInitializable.hpp"

namespace crf {
namespace applications {
namespace personfollower {

class IPersonDetector : public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IPersonDetector() = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;
    virtual cv::Rect2d getPersonBoundingBox() = 0;
    virtual int getImageCenterWidth() = 0;
};

}  // namespace personfollower
}  // namespace applications
}  // namespace crf
