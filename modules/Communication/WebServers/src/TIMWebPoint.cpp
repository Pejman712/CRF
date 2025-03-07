/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <boost/optional.hpp>

#include "WebServers/TIM/TIMWebPoint.hpp"

namespace crf {
namespace communication {
namespace webservers {

TIMWebPoint::TIMWebPoint(const std::shared_ptr<robots::tim::ITIM>& tim,
    std::chrono::duration<float> dataGrabberFrequency,
    std::chrono::duration<float> reconnectionTentativeFrequency) :
    logger_("TIMWebPoint"),
    tim_(tim),
    initialized_(false),
    historyMaxSize_(5),
    positionHistory_(),
    rpHistory_(),
    oxHistory_(),
    tempHistory_(),
    dataGrabberStop_(false),
    dataGrabberFrequency_(dataGrabberFrequency),
    reconnectionTentativeFrequency_(reconnectionTentativeFrequency),
    status_("offline"),
    position_(0),
    velocity_(0),
    battery_(0),
    ischarging_(false),
    economymode_(false),
    ismoving_(false),
    payloadretracted_(false),
    rpvalue_(0),
    oxygen_(0),
    temperature_(0) {
        logger_->debug("CTor");
}

TIMWebPoint::~TIMWebPoint() {
    logger_->debug("DTor");
    if (initialized_) {
        deinitialize();
    }
}

bool TIMWebPoint::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    dataGrabberStop_ = false;
    dataGrabberThread_ = std::thread(&TIMWebPoint::dataGrabber, this);

    initialized_ = true;
    return true;
}

bool TIMWebPoint::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    dataGrabberStop_ = true;
    dataGrabberThread_.join();

    initialized_ = false;
    return tim_->deinitialize();
}

std::string TIMWebPoint::getStatus() {
    nlohmann::json json_;
    json_["status"] = status_;

    if ((status_ == "offline") ||
        (status_ == "error")) {
            return json_.dump();
    }

    json_["position"] = position_;
    json_["velocity"] = velocity_;
    json_["battery"] = battery_;
    json_["ischarging"] = ischarging_;
    json_["economymode"] = economymode_;
    json_["ismoving"] = ismoving_;
    json_["payloadretracted"] = payloadretracted_;
    json_["rpvalue"] = rpvalue_;
    json_["oxygen"] = oxygen_;
    json_["temperature"] = temperature_;

    return json_.dump();
}

std::string TIMWebPoint::getDataHistory() {
    nlohmann::json json_;
    json_["position"] = positionHistory_;
    json_["rp"] = rpHistory_;
    json_["ox"] = oxHistory_;
    json_["temp"] = tempHistory_;

    return json_.dump();
}

std::string TIMWebPoint::getCameraFrame(const std::string& name) {
    if (status_ != "online")
        return std::string();

    if (camerasLatestFrames_.find(name) == camerasLatestFrames_.end()) {
        logger_->warn("Requested name does not exists");
        return std::string();
    }

    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80);

    std::vector<uchar> buf;
    cv::imencode(".jpeg", camerasLatestFrames_[name], buf, compression_params);
    return std::string(buf.begin(), buf.end());
}

bool TIMWebPoint::addCamera(const std::string& name,
    std::shared_ptr<sensors::cameras::ICamera> camera) {
        if (camerasLatestFrames_.find(name) != camerasLatestFrames_.end()) {
            logger_->warn("Camera already added");
            return false;
        }

        cameras_.insert({name, camera});
        camerasLatestFrames_.insert({name, cv::Mat()});
        return true;
}

std::vector<std::string> TIMWebPoint::getCameraNames() {
    std::vector<std::string> cameras;
    for (auto it = cameras_.begin(); it != cameras_.end(); ++it) {
        cameras.push_back(it->first);
    }

    return cameras;
}

bool TIMWebPoint::dataGrabber() {
    reconnectionProcedure();
    while (!dataGrabberStop_) {
        std::this_thread::sleep_for(dataGrabberFrequency_);
        if ((!updateLocalData()) && (!tim_->isConnected())) {
            logger_->warn("Connection lost with TIM, starting auto reconnection procedure");
            reconnectionProcedure();
            continue;
        }

        if (economymode_) continue;
        updateDataHistory();
        cameraFramesGrabber();
    }

    logger_->info("Data grabber exit");
    return true;
}

void TIMWebPoint::reconnectionProcedure() {
    positionHistory_.clear();
    rpHistory_.clear();
    oxHistory_.clear();
    tempHistory_.clear();

    while ((!tim_->isConnected()) && (!dataGrabberStop_)) {
        if (tim_->initialize()) {
            return;
        }
        logger_->warn("Could not reconnect to TIM, trying again in {} seconds",
            std::chrono::duration_cast<std::chrono::seconds>
                (reconnectionTentativeFrequency_).count());
        std::this_thread::sleep_for(reconnectionTentativeFrequency_);
    }
}

bool TIMWebPoint::updateLocalData() {
    auto position = tim_->getPosition();
    auto velocity = tim_->getVelocity();
    auto battery = tim_->getBatteryVoltage();
    auto ischarging = tim_->isCharging();
    auto economyMode = tim_->economyModeActive();
    auto ismoving = tim_->isMoving();
    auto payloadretracted = tim_->allPayloadRetracted();
    auto rpvalue = tim_->getRPValue();
    auto oxygen = tim_->getOxygenPercentage();
    auto temperature = tim_->getTemperature();

    if ((!position) || (!velocity) || (!battery) || (!ischarging) || (!economyMode) ||
        (!ismoving) || (!payloadretracted) || (!rpvalue) || (!oxygen) || (!temperature)) {
            if (!tim_->isConnected()) {
                status_ = "offline";
            } else {
                status_ = "error";
            }
            return false;
    }

    status_ = "online";
    position_ = position.get();
    velocity_ = velocity.get();
    battery_ = battery.get();
    ischarging_ = ischarging.get();
    economymode_ = economyMode.get();
    ismoving_ = ismoving.get();
    payloadretracted_ = payloadretracted.get();
    rpvalue_ = rpvalue.get();
    oxygen_ = oxygen.get();
    temperature_ = temperature.get();
    return true;
}

void TIMWebPoint::updateDataHistory() {
    historyMaxSize_ = ismoving_ ? 100 : 5;

    while (positionHistory_.size() >= historyMaxSize_) {
        positionHistory_.erase(positionHistory_.begin());
        rpHistory_.erase(rpHistory_.begin());
        oxHistory_.erase(oxHistory_.begin());
        tempHistory_.erase(tempHistory_.begin());
    }

    positionHistory_.push_back(position_);
    rpHistory_.push_back(rpvalue_);
    oxHistory_.push_back(oxygen_);
    tempHistory_.push_back(temperature_);
}

void TIMWebPoint::cameraFramesGrabber() {
    for (auto it = cameras_.begin(); it != cameras_.end(); ++it) {
        if (!it->second->initialize()) continue;
        camerasLatestFrames_[it->first] = it->second->getFrame();
        it->second->deinitialize();
    }
}

}  // namespace webservers
}  // namespace communication
}  // namespace crf
