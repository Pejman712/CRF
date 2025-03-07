/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#include <array>
#include <mutex>
#include <iostream>
#include <sophus/se3.hpp>

#include "DsoDevel/PositionObserver.hpp"

namespace {
std::ostream& operator <<(std::ostream& stream, const Sophus::SE3d& se3) {
    stream << "{Rot: [" << se3.angleX() << ", " << se3.angleY() << ", " << se3.angleZ() << "], ";
    const Sophus::SE3d::TranslationMember& trans = se3.translation();
    stream << "Trans: [" << trans[0] << ", " << trans[1] << ", " << trans[2] << "]}";
    return stream;
}
}  // anonymous namespace

namespace crf {
namespace applications {
namespace dsodevel {

PositionObserver::PositionObserver():
    asyncGuard_(), logger_("PositionObserver"), currentPosition_() {
    logger_->debug("CTor");
}

void PositionObserver::publishCamPose(dso::FrameShell* frame, dso::CalibHessian* HCalib) {
    std::lock_guard<std::mutex> lg(asyncGuard_);
    logger_->debug("publishCamPose");
    logger_->debug("camToTrackingRef: {}", frame->camToTrackingRef);
    logger_->debug("camToWorld: {}", frame->camToWorld);
    currentPosition_ = frame->camToWorld;
}

utility::types::TaskPose PositionObserver::getPosition() {
    std::lock_guard<std::mutex> lg(asyncGuard_);
    const Sophus::SE3d::TranslationMember& trans = currentPosition_.translation();
    utility::types::TaskPose pos({static_cast<float>(trans[0]),
        static_cast<float>(trans[1]), static_cast<float>(trans[2]),
        static_cast<float>(currentPosition_.angleX()),
        static_cast<float>(currentPosition_.angleY()),
        static_cast<float>(currentPosition_.angleZ())});
    return pos;
}

}  // namespace dsodevel
}  // namespace applications
}  // namespace crf
