/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Pawel Ptasznik CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#include "ThermalCamera/OptrisPi.hpp"

#include <algorithm>
#include <memory>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace crf {
namespace sensors {
namespace thermalcamera {

OptrisPiImagerClient::OptrisPiImagerClient(std::shared_ptr<evo::IRImager> imager):
    logger_("OptrisPiImagerClient"),
    imager_(imager),
    rawData_(),
    thermalData_(),
    width_(0),
    height_(0) {
    logger_->debug("CTor");
    if (!imager_) {
        logger_->error("Received nullptr imager!");
        return;
    }
    logger_->debug("Setting imager client");
    imager_->setClient(this);
}

void OptrisPiImagerClient::onRawFrame(unsigned char* data, int size) {
    logger_->debug("onRawFrame");
    if (!imager_) return;
    if (!data || size == 0) return;
    logger_->debug("Received frame, size: {}", size);
    imager_->process(data);
    std::lock_guard<std::mutex> guard(lock_);
    rawData_.assign(data, data + size);
}

void OptrisPiImagerClient::onThermalFrame(unsigned short* data, unsigned int w, unsigned int h,  // NOLINT
    evo::IRFrameMetadata meta, void* arg) {
    logger_->debug("onThermalFrame");
    if (!data || w == 0 || h == 0) return;
    std::lock_guard<std::mutex> guard(lock_);
    thermalData_.resize(w*h);
    std::transform(data, data + w*h, thermalData_.begin(),
        [](unsigned short elem) {  // NOLINT
            return (static_cast<float>(elem)-1000.f)/10.f;
        });
    width_ = w;
    height_ = h;
}

void OptrisPiImagerClient::onThermalFrameEvent(unsigned short* data, unsigned int w,  // NOLINT
    unsigned int h, evo::IRFrameMetadata meta, void* arg) {
    logger_->debug("onThermalFrameEvent not supported");
}

void OptrisPiImagerClient::onVisibleFrame(unsigned char* data, unsigned int w, unsigned int h,
    evo::IRFrameMetadata meta, void* arg) {
    logger_->debug("onVisibleFrame not supported");
}

void OptrisPiImagerClient::onVisibleFrameEvent(unsigned char* data, unsigned int w, unsigned int h,
    evo::IRFrameMetadata meta, void* arg) {
    logger_->debug("onVisibleFrameEvent not supported");
}

void OptrisPiImagerClient::onFlagStateChange(evo::EnumFlagState flagstate, void* arg) {
    logger_->debug("onFlagStateChange not supported");
}

std::vector<unsigned char> OptrisPiImagerClient::getRawData() {
    std::lock_guard<std::mutex> guard(lock_);
    return rawData_;
}

std::vector<float> OptrisPiImagerClient::getThermalData() {
    std::lock_guard<std::mutex> guard(lock_);
    return thermalData_;
}

unsigned int OptrisPiImagerClient::getWidth() {
    std::lock_guard<std::mutex> guard(lock_);
    return width_;
}
unsigned int OptrisPiImagerClient::getHeight() {
    std::lock_guard<std::mutex> guard(lock_);
    return height_;
}

OptrisPi::OptrisPi(std::shared_ptr<IPC> ipc, std::shared_ptr<IOptrisDeviceFactory> factory):
    logger_("OptrisPi"),
    ipc_(ipc),
    factory_(factory),
    device_(nullptr),
    callbackClient_(nullptr),
    captureImageThread_() {
    logger_->debug("CTor");
    logger_->info("OpenCV_{}.{}", CV_MAJOR_VERSION, CV_MINOR_VERSION);
}

OptrisPi::~OptrisPi() {
    logger_->debug("DTor");
    deinitialize();
}

bool OptrisPi::initialize() {
    logger_->debug("initialize");
    if (device_) {
        logger_->warn("Already initialized!");
        return false;
    }
    logger_->info("Going to create a device");
    device_ = factory_->createDevice();

    if (!device_) {
        logger_->error("Failed to create a device!");
        return false;
    }

    logger_->info("Device: {}, {}, {}",
        device_->getSerial(), device_->getWidth(), device_->getHeight());
    callbackClient_.reset(new OptrisPiImagerClient(factory_->createImager(
        device_->getFrequency(), device_->getWidth(), device_->getHeight())));
    logger_->debug("Setting callback client for device");
    device_->setClient(callbackClient_.get());
    logger_->debug("Start streaming");
    int success = device_->startStreaming();
    logger_->debug("Started streaming with result: {}", success);
    captureImageThread_ = std::thread([this](){
        // don't use any mutex, "run" is a blocking operation
        device_->run();
        logger_->info("Capture thread finished");
    });
    if (!ipc_->open()) {
        logger_->warn("Cannot open IPC, publishFrame will fail");
    }
    return true;
}

bool OptrisPi::deinitialize() {
    logger_->debug("deinitialize");
    if (!device_) {
        logger_->warn("Already deinitialized!");
        return false;
    }
    device_->exit();
    logger_->debug("Exited blocking mode");
    if (captureImageThread_.joinable()) {
        captureImageThread_.join();
        logger_->debug("Thread joint");
    }
    device_->stopStreaming();
    logger_->debug("Stopped streaming");
    device_.reset();
    logger_->debug("Device destroyed");
    callbackClient_.reset();
    return true;
}

cv::Mat OptrisPi::getFrame() {
    if (!device_ || !callbackClient_) {
        logger_->warn("No device or empty client!");
        return cv::Mat();
    }
    std::vector<float> thermalData = callbackClient_->getThermalData();
    if (thermalData.empty()) {
        logger_->warn("Empty thermal data!");
        return cv::Mat();
    }
    cv::Mat frame(callbackClient_->getHeight(), callbackClient_->getWidth(), CV_32F);
    for (int i = 0; i < callbackClient_->getHeight(); i++) {
        for (int j = 0; j < callbackClient_->getWidth(); j++) {
            frame.at<float>(i, j) = thermalData[i*callbackClient_->getWidth() + j];
        }
    }
    return frame;
}

bool OptrisPi::publishFrame(const cv::Mat& frame) {
    if (frame.empty()) {
        return false;
    }
    Packets::ThermalCameraPacket cameraPacket;
    cameraPacket.temperatures = frame;
    return ipc_->write(cameraPacket.serialize(), cameraPacket.getHeader());
}

bool OptrisPi::showFrame() {
    if (!device_) {
        logger_->warn("No device!");
        return false;
    }
    std::vector<float> thermalData = callbackClient_->getThermalData();
    if (thermalData.size() == 0) {
        logger_->warn("Empty thermal frame");
        return false;
    }
    showOpenCvFrames(&thermalData);
    cv::waitKey();
    return true;
}

bool OptrisPi::streamVideo() {
    if (!device_) {
        logger_->warn("No device!");
        return false;
    }
    std::vector<float> thermalData = callbackClient_->getThermalData();
    if (thermalData.size() == 0) {
        logger_->warn("Empty thermal frame");
        return false;
    }
    unsigned int sleepFor = 1000/device_->getFrequency();
    while (true) {
        auto max = std::max_element(thermalData.begin(), thermalData.end());
        auto min = std::min_element(thermalData.begin(), thermalData.end());
        logger_->debug("Max: {}, Min: {}", *max, *min);
        showOpenCvFrames(&thermalData);
        if (cv::waitKey(1) != -1) break;
        thermalData = callbackClient_->getThermalData();
        if (thermalData.size() == 0) {
            logger_->warn("Sth not OK. Got empty frame");
            break;
        }
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(sleepFor));
    }
}

void OptrisPi::showOpenCvFrames(std::vector<float>* thermalData) {
    /*
     * WARNING! this kind of matrix construction is only safe if you can guarantee consistency
     * of the data pointed by thermalData->data();
     * cv::Mat seems to be too lazy to actually copy it from the array
     */
    cv::Mat frameForColormap(callbackClient_->getHeight(), callbackClient_->getWidth(), CV_32F,
        thermalData->data());
    frameForColormap.convertTo(frameForColormap, CV_8UC1);
    cv::equalizeHist(frameForColormap, frameForColormap);
    cv::applyColorMap(frameForColormap, frameForColormap, cv::COLORMAP_JET);
    std::vector<float> normalizedData;
    cv::normalize(*thermalData, normalizedData, 0, 1, cv::NORM_MINMAX);
    cv::Mat frameForGrey(callbackClient_->getHeight(), callbackClient_->getWidth(), CV_32F,
        normalizedData.data());
    cv::imshow("FrameColorMap", frameForColormap);
    cv::imshow("FrameGrey", frameForGrey);
}

}  // namespace thermalcamera
}  // namespace sensors
}  // namespace crf
