/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */


#include "DsoDevel/DsoDevel.hpp"

#include <sys/stat.h>

#include <string>
#include <fstream>
#include <memory>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include <opencv2/opencv.hpp>

#include "DirectSparseOdometry/util/Undistort.hpp"
#include "DirectSparseOdometry/IOWrapper/OutputWrapper/PangolinDSOViewer.hpp"
#include "DirectSparseOdometry/FullSystem/FullSystem.hpp"

#include "DsoDevel/PositionObserver.hpp"

namespace crf {
namespace applications {
namespace dsodevel {

DsoDevel::DsoDevel(const std::string& configFileName, std::shared_ptr<cv::VideoCapture> camera):
    stop_(true),
    asyncGuard_(),
    logger_("DsoDevel"),
    camera_(camera) {
    logger_->debug("CTor");
    logger_->debug("OpenCV version: {}.{}", CV_MAJOR_VERSION, CV_MINOR_VERSION);
    std::ifstream configFile(configFileName);
    if (!configFile.is_open()) {
        logger_->error("Failed to open config file: {}", configFileName);
        throw std::runtime_error("Failed to open config file: " + configFileName);
    }
    if (!camera_->isOpened()) {
        logger_->error("Camera is not opened");
        throw std::runtime_error("Camera is not opened");
    }
    nlohmann::json jsonConfig;
    try {
        configFile >> jsonConfig;
    } catch (const std::exception& e) {
        logger_->error("Json parse error: {}", e.what());
        throw;
    }
    configFile.close();
    try {
        vignette_ = jsonConfig.at("vignette");
        cameraConfig_ = jsonConfig.at("cameraConfig");
        gammaCalib_ = jsonConfig.at("gammaCalib");
    } catch (const std::exception& e) {
        logger_->error("Exception thrown while decoding the config file: {}", e.what());
        throw;
    }
    for (const auto& fileName : {vignette_, cameraConfig_, gammaCalib_}) {
        std::ifstream  f(fileName);
        if (!f.good()) {
            logger_->error("{} doesn't exist or is not accessible", fileName);
            throw std::runtime_error(fileName + "doesn't exist or is not accessible");
        }
    }
    logger_->info("Going to create Undistort ... ");
    undistorter_ = dso::Undistort::getUndistorterForFile(
        cameraConfig_, gammaCalib_, vignette_);
    if (undistorter_->photometricUndist == nullptr) {
        logger_->error("Some problems with photometric calibration");
        throw std::runtime_error("Some problems with photometric calibration");
    }
    imgWidth_ = undistorter_->getSize()[0];
    imgHeight_ = undistorter_->getSize()[1];
    logger_->info("Going to construct PositionObserver ... ");
    positionObserver_.reset(new PositionObserver);
    logger_->info("Going to set global variables of DSO lib ... ");
    dso::setGlobalCalib(imgWidth_, imgHeight_, undistorter_->getK().cast<float>());
    logger_->debug("Object constructed!");
}

DsoDevel::~DsoDevel() {
    logger_->debug("DTor");
    if (dsoThread_.joinable()) {
        stopDso();
    }
}

bool DsoDevel::asyncRunDso(std::shared_ptr<dso::IOWrap::Output3DWrapper> outputWrapper) {
    logger_->info("Going to run async DSO");
    if (dsoThread_.joinable()) {
        logger_->warn("Async DSO job is already running. Execute stopDso to stop it.");
        return false;
    }
    stop_ = false;
    dsoThread_ = std::thread(std::bind(&DsoDevel::dsoLoop, this, outputWrapper));
    return true;
}

bool DsoDevel::stopDso() {
    std::lock_guard<std::mutex> lg(asyncGuard_);
    logger_->debug("stopDso");
    bool status = true;
    if (stop_) {
        logger_->warn("Attempt to stop already stopped system");
        status = false;
    }
    stop_ = true;
    if (dsoThread_.joinable()) {
        logger_->debug("Waiting for thread to join");
        dsoThread_.join();
    } else {
        logger_->warn("Thread is not joinable");
        status = false;
    }
    logger_->debug("Going to return {}", status);
    return status;
}

utility::types::TaskPose DsoDevel::getPosition() {
    return positionObserver_->getPosition();
}

void DsoDevel::dsoLoop(std::shared_ptr<dso::IOWrap::Output3DWrapper> outputWrapper) {
    logger_->info("Going to create FullSystem ... ");
    std::unique_ptr<dso::FullSystem> fullSystem(new dso::FullSystem());

    fullSystem->linearizeOperation = false;
    logger_->debug("Going to push PositionObserver");
    fullSystem->outputWrapper.push_back(positionObserver_);
    if (outputWrapper != nullptr) {
        logger_->info("Going to push output wrapper ... ");
        fullSystem->outputWrapper.push_back(outputWrapper);
    } else {
        logger_->warn("Come on ... you invoked this method with outputWrapper == nullptr ...");
    }

    logger_->info("Going to setGammaFunction ... ");
    fullSystem->setGammaFunction(undistorter_->photometricUndist->getG());

    logger_->info("Let's go !!!");
    bool isKF = false;
    int frameId = 0;
    cv::Mat frame, temp, activeFrame;
    while (!stop_) {
        logger_->debug("Getting frame from the camera");
        if (!camera_->read(frame)) {
            logger_->warn("Failed to read next frame from the VideoCapture device");
            std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(10));
            continue;
        }
        cv::cvtColor(frame, temp, CV_BGR2GRAY);
        temp.convertTo(activeFrame, CV_8U);

        logger_->debug("Creating MinimalImageB");
        dso::MinimalImageB minImg(static_cast<int>(activeFrame.cols),
                                  static_cast<int>(activeFrame.rows),
            (unsigned char*)activeFrame.data);

        logger_->debug("Getting ImageAndExposure from undistort");
        dso::ImageAndExposure* undistImage = undistorter_->undistort(&minImg, 1);

        logger_->debug("Setting active frame");
        isKF = fullSystem->addActiveFrame(undistImage, frameId);

        if (fullSystem->isLost) {
            std::lock_guard<std::mutex> lg(asyncGuard_);
            logger_->warn("System got lost! Going to stop async routine and destroy FullSystem.");
            stop_ = true;
        }

        frameId++;
        delete undistImage;
    }

    for (auto ow : fullSystem->outputWrapper) {
        ow->join();
    }
    logger_->info("Finito!");
}


}  // namespace dsodevel
}  // namespace applications
}  // namespace crf
