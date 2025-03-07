/* © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023
 *
 * ====================================================================
*/

#pragma once

#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include <boost/program_options.hpp>
#include "crf/expected.hpp"
#include "EventLogger/EventLogger.hpp"
#include "ViconAPI/IViconApi.hpp"
#include "ViconAPI/ViconApi.hpp"
#include "MotionCapture/IMotionCapture.hpp"
#include "MotionCapture/MotionCaptureMarker.hpp"

namespace crf::sensors::motioncapture {

/**
 * @ingroup group_vicon
 * @brief Implementation of the Vicon Motion Capture System.
 * 
 */
class ViconMotionCapture : public IMotionCapture {
 public:
    ViconMotionCapture(const std::string& hostName, const nlohmann::json& config,
        std::shared_ptr<crf::communication::viconapi::IViconApi> viconApi = nullptr);
    ~ViconMotionCapture();
    bool initialize() override;
    bool deinitialize() override;
    crf::expected<std::vector<std::string>> getObjectNames() override;
    crf::expected<crf::utility::types::TaskPose> getObjectPose(
        const std::string objectName) override;
    crf::expected<std::vector<MotionCaptureMarker>> getObjectMarkers(
        const std::string objectName) override;

 private:
    enum class StreamingType {
        ServerPush = 0,
        ClientPull = 1,
        ClientPullPreFetch = 2
    };
    enum class AxisMappingType {
        ZUp = 0,
        XUp = 1,
        YUp = 2
    };
    std::shared_ptr<crf::communication::viconapi::IViconApi> client_;
    std::string host_;
    crf::utility::logger::EventLogger logger_;
    StreamingType streamType_;
    AxisMappingType axisMapping_;
    unsigned int clientBufferSize_;
    unsigned int timeout_;
    bool lightweight_;
    bool objectsOnly_;
    bool initialized_;
    bool connect();
    bool waitFrame();
    bool parse(const nlohmann::json& configFile);
    crf::Code evaluate(ViconDataStreamSDK::CPP::Result::Enum result);
};

}  // namespace crf::sensors::motioncapture
