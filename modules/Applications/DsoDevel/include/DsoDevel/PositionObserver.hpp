/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#pragma once

#include <array>
#include <mutex>
#include <sophus/se3.hpp>

#include "DirectSparseOdometry/IOWrapper/Output3DWrapper.hpp"
#include "DirectSparseOdometry/util/FrameShell.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Types/TaskTypes/TaskPose.hpp"

namespace crf {
namespace applications {
namespace dsodevel {

class PositionObserver: public dso::IOWrap::Output3DWrapper {
 public:
    PositionObserver();
    ~PositionObserver() override {}
    void publishCamPose(dso::FrameShell* frame, dso::CalibHessian* HCalib) override;
    /*
     * Returns current estimated position, with the reference to the starting point in the form:
     *   [x, y, z, roll, pitch, yaw]
     * Now, beware, this might not be obvious for everyone. This piece of code will work just fine:
     *     auto myCurrentPos = posObserver->getPosition();
     *     logger->debug("My XYZ: {} {} {}", myCurrentPos[0], myCurrentPos[1], myCurrentPos[2]);
     * However, this doesn't guarantee to give you the correct result:
     *     logger->debug("My XYZ: {} {} {}",
     *         posObserver->getPosition()[0],
     *         posObserver->getPosition()[1],
     *         posObserver->getPosition()[2]);
     * because PositionObserver is updating the position estimate asynchronously.
     */
    utility::types::TaskPose getPosition();
 private:
    std::mutex asyncGuard_;
    utility::logger::EventLogger logger_;
    Sophus::SE3d currentPosition_;
};

}  // namespace dsodevel
}  // namespace applications
}  // namespace crf
