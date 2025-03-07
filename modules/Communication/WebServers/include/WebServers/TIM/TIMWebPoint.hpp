#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <restbed>
#include <string>
#include <thread>
#include <vector>

#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"
#include "TIM/ITIM.hpp"
#include "Cameras/ICamera.hpp"

namespace crf {
namespace communication {
namespace webservers {

class TIMWebPoint : public utility::commoninterfaces::IInitializable {
 public:
    TIMWebPoint() = delete;
    TIMWebPoint(const std::shared_ptr<robots::tim::ITIM>&,
        std::chrono::duration<float> dataGrabberFrequency = std::chrono::milliseconds(500),
        std::chrono::duration<float> reconnectionTentativeFrequency = std::chrono::seconds(10));
    TIMWebPoint(TIMWebPoint&&) = delete;
    TIMWebPoint(const TIMWebPoint&) = delete;
    ~TIMWebPoint() override;

    bool initialize() override;
    bool deinitialize() override;

    std::string getStatus();
    std::string getDataHistory();
    std::string getCameraFrame(const std::string& name);

    bool addCamera(const std::string& name, std::shared_ptr<sensors::cameras::ICamera> camera);
    std::vector<std::string> getCameraNames();

 private:
    utility::logger::EventLogger logger_;
    const std::shared_ptr<robots::tim::ITIM> tim_;
    bool initialized_;

    size_t historyMaxSize_;
    std::vector<float> positionHistory_;
    std::vector<float> rpHistory_;
    std::vector<float> oxHistory_;
    std::vector<float> tempHistory_;

    std::map<std::string, std::shared_ptr<sensors::cameras::ICamera> > cameras_;
    std::map<std::string, cv::Mat> camerasLatestFrames_;

    std::atomic<bool> dataGrabberStop_;
    std::thread dataGrabberThread_;
    const std::chrono::duration<float> dataGrabberFrequency_;
    const std::chrono::duration<float> reconnectionTentativeFrequency_;
    bool dataGrabber();

    void reconnectionProcedure();
    bool updateLocalData();
    void updateDataHistory();
    void cameraFramesGrabber();

    std::string status_;
    float position_;
    float velocity_;
    float battery_;
    bool ischarging_;
    bool economymode_;
    bool ismoving_;
    bool payloadretracted_;
    float rpvalue_;
    float oxygen_;
    float temperature_;
};

}  // namespace webservers
}  // namespace communication
}  // namespace crf
