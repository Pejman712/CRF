/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <memory>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <deque>
#include <nlohmann/json.hpp>
#include <thread>
#include <iostream>
#include <csignal>
#include <exception>
#include <future>
#include <assert.h>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/time.h>

#include "EventLogger/EventLogger.hpp"
#include "Laser/VelodyneHDL/VelodyneHDLLaser.hpp"
#include "Laser/VelodyneHDL/VelodyneHDLLaserConfiguration.hpp"
#include "Laser/VelodyneHDL/VelodyneHDLPacketDriver.hpp"
#include "Laser/VelodyneHDL/VelodyneHDLPacketDecoder.hpp"

namespace crf {
namespace sensors {
namespace laser {

bool VelodyneHDLLaser::readingPointCloud_{true};
bool VelodyneHDLLaser::isInitialized_{false};

VelodyneHDLLaser::VelodyneHDLLaser(const nlohmann::json& laserJSON):
    laserJSON_(laserJSON),
    logger_("VelodyneHDLLaser"),
    configuration_(new VelodyneHDLLaserConfiguration()),
    networkConfig_{},
    driver_{},
    decoder_{} {
    logger_->debug("CTor");

    if (!configuration_->parse(laserJSON_)) {
        throw std::runtime_error("Failed to read the configuration file");
    }
    networkConfig_ = configuration_->getNetworkConfiguration();
}

VelodyneHDLLaser::~VelodyneHDLLaser() {
    logger_->debug("DTor");
}

bool VelodyneHDLLaser::initialize() {
    logger_->debug("initialize");
    if (isInitialized_) {
        logger_->debug("Could not initialize laser because already initialized");
        return false;
    }
    if (!driver_.InitPacketDriver(static_cast<unsigned int> (networkConfig_.udpPort))) {
        logger_->debug("Failed to initialize Velodyne HDL laser");
        return false;
    }
    decoder_.SetCorrectionsFile(configuration_->getCorrectionParametersFilePath());
    if (getInternalPointCloud() == nullptr) {
        logger_->error("GetPointCloud has returned nullptr");
        return false;
    }
    logger_->info("Velodyne HDL laser initialized");
    isInitialized_ = true;
    return true;
}

bool VelodyneHDLLaser::deinitialize() {
    logger_->debug("deinitialize");
    if (!isInitialized_) {
        logger_->warn("Could not deinitialize laser because not initialized");
        return false;
    }
    isInitialized_ = false;
    return true;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr VelodyneHDLLaser::getPointCloud() {
    logger_->debug("getPointCloud");
    if (!isInitialized_) {
        logger_->warn("Could not take a PC because not initialized");
        return nullptr;
    }
    return getInternalPointCloud();
}

std::shared_ptr<crf::sensors::laser::LaserConfiguration> VelodyneHDLLaser::getConfiguration() {
    logger_->debug("getConfiguration");
    return configuration_;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr VelodyneHDLLaser::getInternalPointCloud() {
    logger_->debug("getInternalPointCloud");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr returnValue = nullptr;

    std::string* data = new std::string();
    unsigned int* dataLength = new unsigned int();
    VelodyneHDLPacketDecoder::HDLFrame latestFrame;

    std::promise<void> exitSignal;
    std::future<void> futureObj = exitSignal.get_future();

    readingPointCloud_ = true;
    std::thread timerThread(&timerFunction, std::move(futureObj));

    for (int i = 0; i < 100; i++) {
        driver_.GetPacket(data, dataLength);
    }
    while (true) {
        if (!driver_.GetPacket(data, dataLength)) {
            continue;
        }
        if (!decoder_.DecodePacket(data, dataLength)) {
            logger_->warn("Unable to get packet");
            break;
        }
        if (!decoder_.GetLatestFrame(&latestFrame)) {
            continue;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudGlobal(new pcl::PointCloud<pcl::PointXYZI>);
        for (size_t i = 0; i < latestFrame.x.size(); i++) {
            pcl::PointXYZI point;
            point.x = latestFrame.x.at(i);
            point.y = latestFrame.y.at(i);
            point.z = latestFrame.z.at(i);
            point.intensity = latestFrame.intensity.at(i);

            cloudGlobal->push_back(point);
        }
        logger_->debug("PointCloud size: {}", cloudGlobal->points.size());
        cloudGlobal->width = static_cast<int>(cloudGlobal->points.size());
        cloudGlobal->height = 1;

        if (cloudGlobal->empty()) {
            logger_->error("The cloudGlobal is empty");
            returnValue = nullptr;
            break;
        }
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::copyPointCloud(*cloudGlobal, *pointCloud);
        if (pointCloud->empty()) {
            logger_->error("The pointCloud is empty");
            returnValue = nullptr;
            break;
        }
        returnValue = pointCloud;
        break;
    }
    readingPointCloud_ = false;
    exitSignal.set_value();
    timerThread.join();
    delete data;
    delete dataLength;

    return returnValue;
}

void VelodyneHDLLaser::timerFunction(std::future<void> futureObj) {
    while (futureObj.wait_for(std::chrono::milliseconds(1000)) != std::future_status::timeout) {
        if (!readingPointCloud_)
            break;
    }
    if (readingPointCloud_) {
        isInitialized_ = false;
        std::raise(SIGTERM);
    }
}

}  // namespace laser
}  // namespace sensors
}  // namespace crf
