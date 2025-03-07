/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

#include "DirectSparseOdometry/util/Undistort.hpp"
#include "DirectSparseOdometry/IOWrapper/Output3DWrapper.hpp"
#include "EventLogger/EventLogger.hpp"
#include "Types/TaskTypes/TaskPose.hpp"

#include "DsoDevel/PositionObserver.hpp"

namespace crf {
namespace applications {
namespace dsodevel {

class DsoDevel {
 public:
    DsoDevel(const std::string& configFileName,
        std::shared_ptr<cv::VideoCapture> camera);
    DsoDevel(const DsoDevel& other) = delete;
    /*
     * Well even though, technically it should be possible to somehow gracefully move the
     * object of this class, I just do not want to mess with that. Especially, I do not
     * want anyone ELSE trying to copy/move these objects.
     */
    DsoDevel(DsoDevel&& other) = delete;
    virtual ~DsoDevel();
    bool asyncRunDso(std::shared_ptr<dso::IOWrap::Output3DWrapper> outputWrapper);
    bool stopDso();
    /*
     * Returns current estimated position, with the reference to the starting point in the form:
     *   [x, y, z, roll, pitch, yaw]
     * Please also read the docs in PositionObserver.hpp
     */
    utility::types::TaskPose getPosition();

 private:
    void dsoLoop(std::shared_ptr<dso::IOWrap::Output3DWrapper> outputWrapper);
    bool stop_;
    std::mutex asyncGuard_;
    utility::logger::EventLogger logger_;
    std::string vignette_;
    std::string cameraConfig_;
    std::string gammaCalib_;
    std::shared_ptr<cv::VideoCapture> camera_;
    std::unique_ptr<dso::Undistort> undistorter_;
    std::shared_ptr<PositionObserver> positionObserver_;
    int imgWidth_;
    int imgHeight_;
    std::thread dsoThread_;
};


}  // namespace dsodevel
}  // namespace applications
}  // namespace crf
