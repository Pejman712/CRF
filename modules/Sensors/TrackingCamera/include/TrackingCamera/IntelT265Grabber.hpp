#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <vector>
#include <memory>
#include <string>

#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>
#include <nlohmann/json.hpp>
#include <boost/optional.hpp>

#include "EventLogger/EventLogger.hpp"
#include "TrackingCamera/ITrackingCamera.hpp"

namespace crf {
namespace sensors {
namespace trackingcamera {

class IntelT265Grabber: public ITrackingCamera {
 public:
    explicit IntelT265Grabber(const nlohmann::json& configCamera);
    IntelT265Grabber() = delete;
    IntelT265Grabber(const IntelT265Grabber& other) = delete;
    IntelT265Grabber(IntelT265Grabber&& other) = delete;
    ~IntelT265Grabber() override;
    bool initialize() override;
    bool deinitialize() override;
    TrackingData getTrackingData() override;
    std::vector<cv::Mat> getVideoFrames() override;
    std::vector<cv::Mat> getRawVideoFrames() override;
    bool feedBaseVelocity(const std::array<float, 3>& velocity) override;
    bool resetPose() override;
    boost::optional<rs2_intrinsics> getCameraIntrinsics(bool cameraSelection) override;
    boost::optional<rs2_extrinsics> getCameraExtrinsics(bool cameraSelection) override;

 private:
    utility::logger::EventLogger logger_;
    nlohmann::json configCamera_;
    bool isRunning_;
    rs2::config cfg_;
    rs2::pipeline pipe_;
    std::shared_ptr<rs2::wheel_odometer> odometer_;
    void configureStreams();
    void initializeStreams(const rs2::pipeline_profile &profile);
    bool initializeOdometryInput(const std::string &calib_odom_file);

    struct StreamsParameters {
        rs2_intrinsics cam1_intrin;
        rs2_intrinsics cam2_intrin;
        rs2_extrinsics cam1_extrin;
        rs2_extrinsics cam2_extrin;
    };
    StreamsParameters streams_;
};

}   // namespace trackingcamera
}   // namespace sensors
}   // namespace cern
