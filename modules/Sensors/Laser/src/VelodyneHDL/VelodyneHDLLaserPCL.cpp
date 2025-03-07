/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Carlos Prados Sesmero & Julia Kabalar EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <csignal>
#include <iostream>

#include <nlohmann/json.hpp>
#include <pcl/common/io.h>
#include <pcl/common/time.h>
#include <boost/function.hpp>

#include "EventLogger/EventLogger.hpp"
#include "Laser/VelodyneHDL/VelodyneHDLLaserPCL.hpp"
#include "Laser/VelodyneHDL/VelodyneHDLLaserConfiguration.hpp"

#define TWO_TURNS 2
#define RECEIVER_WAITING_TIME_MILLISEC 500
#define MAX_TIME_WAITING 10000.0

namespace crf {
namespace sensors {
namespace laser {

VelodyneHDLLaserPCL::VelodyneHDLLaserPCL(const nlohmann::json& laserJSON):
    laserJSON_(laserJSON),
    logger_("VelodyneHDLLaserPCL"),
    isInitialized_(false),
    configuration_(new VelodyneHDLLaserConfiguration()),
    cloud_(new pcl::PointCloud<pcl::PointXYZI>) {
    logger_->debug("CTor");
    if (!configuration_->parse(laserJSON_)) {
        throw std::runtime_error("Failed to read the configuration file");
    }
    VelodyneHDLLaserNetworkConfiguration networkConfig = configuration_->getNetworkConfiguration();
    grabber_.reset(new pcl::HDLGrabber(
        boost::asio::ip::address_v4::from_string(networkConfig.ipAddress),
        networkConfig.udpPort,
        configuration_->getCorrectionParametersFilePath()));
}

VelodyneHDLLaserPCL::~VelodyneHDLLaserPCL() {
    logger_->debug("DTor");
    deinitialize();
}

bool VelodyneHDLLaserPCL::initialize() {
    logger_->debug("initialize");
    if (isInitialized_) {
        logger_->debug("Could not initialize laser because already initialized");
        return false;
    }
    std::function<void(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)> cloudCB =
        [this] (const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) {cloudCallback(cloud);};
    cloudConnection_ = grabber_->registerCallback(cloudCB);
    grabber_->start();
    crf::sensors::laser::LaserParameters parameters = configuration_->getLaserParameters();
    grabber_->setMinimumDistanceThreshold(parameters.minRange);
    grabber_->setMaximumDistanceThreshold(parameters.maxRange);
    if (getInternalPointCloud() == nullptr) {
        logger_->error("GetPointCloud has returned nullptr");
        return false;
    }
    logger_->info("Initialized {}", grabber_->getName());
    isInitialized_ = true;
    return true;
}

bool VelodyneHDLLaserPCL::deinitialize() {
    logger_->debug("deinitialize");
    if (!isInitialized_) {
        logger_->warn("Could not deinitialize laser because not initialized");
        return false;
    }
    grabber_->stop();
    cloudConnection_.disconnect();
    isInitialized_ = false;
    return true;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr VelodyneHDLLaserPCL::getPointCloud() {
    logger_->debug("getPointCloud");
    if (!isInitialized_) {
        logger_->warn("Could not take a PC because not initialized");
        return nullptr;
    }
    return getInternalPointCloud();
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr VelodyneHDLLaserPCL::getInternalPointCloud() {
    logger_->debug("getInternalPointCloud");

    pcl::PointCloud<pcl::PointXYZI> cloudGlobal;
    auto start = std::chrono::system_clock::now();
    for (unsigned int i = 0; i < TWO_TURNS;) {
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud;
        if (cloudMutex_.try_lock()) {
            cloud_.swap(cloud);
            cloudMutex_.unlock();
        }
        if (cloud) {
            if (i == 1) cloudGlobal = *cloud;
            i++;
        }
        std::chrono::duration<float, std::milli> time = std::chrono::system_clock::now() - start;
        if (time.count() >= RECEIVER_WAITING_TIME_MILLISEC) {
            logger_->error("The point cloud is taking too much time to be received");
            return nullptr;
        }
        std::this_thread::sleep_for(configuration_->getScanIntervalUS());
    }
    if (cloudGlobal.empty()) {
        logger_->error("The cloudGlobal is empty");
        return nullptr;
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(cloudGlobal, *pointCloud);  // Convert to PointXYZRGBA type
    if (pointCloud->empty()) {
        logger_->error("The pointCloud is empty");
        return nullptr;
    }
    return pointCloud;
}

std::shared_ptr<crf::sensors::laser::LaserConfiguration> VelodyneHDLLaserPCL::getConfiguration() {
    logger_->debug("getConfiguration");
    return configuration_;
}

void VelodyneHDLLaserPCL::cloudCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) {
    logger_->debug("cloudCallback");
    std::lock_guard<std::mutex> lock(cloudMutex_);
    cloud_ = cloud;
}

}  // namespace laser
}  // namespace sensors
}  // namespace crf
