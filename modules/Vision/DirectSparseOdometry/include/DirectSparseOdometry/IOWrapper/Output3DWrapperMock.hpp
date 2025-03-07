#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <vector>
#include <utility>
#include <map>
#include <functional>

#include "DirectSparseOdometry/IOWrapper/Output3DWrapper.hpp"

namespace dso {
namespace IOWrap {

class Output3DWrapperMock : public Output3DWrapper {
 public:
  MOCK_METHOD1(publishGraph,
      void(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>,
      Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i> > > &connectivity));
  MOCK_METHOD3(publishKeyframes,
      void(std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib));  //NOLINT
  MOCK_METHOD2(publishCamPose,
      void(FrameShell* frame, CalibHessian* HCalib));
  MOCK_METHOD1(pushLiveFrame,
      void(FrameHessian* image));
  MOCK_METHOD1(pushDepthImage,
      void(MinimalImageB3* image));
  MOCK_METHOD0(needPushDepthImage,
      bool());
  MOCK_METHOD2(pushDepthImageFloat,
      void(MinimalImageF* image, FrameHessian* KF));
  MOCK_METHOD0(join,
      void());
  MOCK_METHOD0(reset,
      void());
};

}  // namespace IOWrap
}  // namespace dso

