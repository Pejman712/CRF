/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include <vector>

#include <opencv2/opencv.hpp>

#include "TrackingCamera/ITrackingCamera.hpp"

namespace crf {
namespace sensors {
namespace trackingcamera {

class TrackingCameraMock : public ITrackingCamera {
 public:
  MOCK_METHOD0(initialize,
      bool());
  MOCK_METHOD0(deinitialize,
      bool());
  MOCK_METHOD0(getTrackingData,
      TrackingData());
  MOCK_METHOD0(getVideoFrames,
      std::vector<cv::Mat>());
  MOCK_METHOD0(getRawVideoFrames,
      std::vector<cv::Mat>());
  MOCK_METHOD1(feedBaseVelocity,
      bool(const std::array<float, 3>& velocity));
  MOCK_METHOD0(resetPose,
      bool());
  MOCK_METHOD1(getCameraIntrinsics,
      boost::optional<rs2_intrinsics>(bool cameraSelection));
  MOCK_METHOD1(getCameraExtrinsics,
      boost::optional<rs2_extrinsics>(bool cameraSelection));
};

}  // namespace trackingcamera
}  // namespace sensors
}  // namespace crf

