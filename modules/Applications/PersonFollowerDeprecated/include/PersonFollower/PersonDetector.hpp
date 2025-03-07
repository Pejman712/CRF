#pragma once
/* Â© Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <opencv2/core.hpp>

#include "EventLogger/EventLogger.hpp"
#include "ObjectDetection/IObjectDetector.hpp"
#include "PersonFollower/IPersonDetector.hpp"
#include "Cameras/ICamera.hpp"

namespace crf {
namespace applications {
namespace personfollower {

class PersonDetector: public IPersonDetector {
 public:
    PersonDetector(std::shared_ptr<crf::sensors::cameras::ICamera> camera,
        std::shared_ptr<crf::vision::objectDetection::IObjectDetector> detector);
    PersonDetector(const PersonDetector& other) = delete;
    PersonDetector(PersonDetector&& other) = delete;
    PersonDetector() = delete;
    ~PersonDetector() override;
    bool initialize() override;
    bool deinitialize() override;
    cv::Rect2d getPersonBoundingBox() override;
    int getImageCenterWidth() override;
 private:
    cv::Rect2d requestPersonBoundingBoxFromServer(cv::Mat colorFrame);

    std::shared_ptr<crf::sensors::cameras::ICamera> camera_;
    std::shared_ptr<crf::vision::objectDetection::IObjectDetector> detector_;
    bool initialized_;
    utility::logger::EventLogger logger_;
};

}  // namespace personfollower
}  // namespace applications
}  // namespace crf
