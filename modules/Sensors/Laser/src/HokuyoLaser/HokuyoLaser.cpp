/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Julia Kabalar EN/SMM/MRO
 * 
 *  ==================================================================================================
 */

#include <string>
#include <vector>
#include <memory>
#include <thread>

#include <urg_c/urg_utils.h>

#include "Laser/HokuyoLaser/HokuyoLaser.hpp"
#include "VisionUtility/PointCloud/PointConverter.hpp"

namespace crf {
namespace sensors {
namespace laser {

HokuyoLaser::HokuyoLaser(LaserConnectionType connectionType, const std::string& deviceOrAdress,
    unsigned int baudrateOrPort):
    laserConfig_(new LaserConfiguration()),
    connectionType_(connectionType),
    deviceOrAdress_(deviceOrAdress),
    baudrateOrPort_(baudrateOrPort),
    initialized_(false),
    logger_("HokuyoLaser") {
    logger_->debug("CTor");
}

HokuyoLaser::~HokuyoLaser() {
    logger_->debug("DTor");
    deinitialize();
}

bool HokuyoLaser::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->debug("Already initialized");
        return true;
    }
    urg_connection_type_t type;
    switch (connectionType_) {
        case LaserConnectionType::Ethernet:
            type = URG_ETHERNET;
            break;
        case LaserConnectionType::Serial:
            type = URG_SERIAL;
            break;
        default:
            logger_->error("Connection type unsupported");
            return false;
    }
    int ret = urg_open(&urg_, type, deviceOrAdress_.c_str(),
        static_cast<long int>(baudrateOrPort_));  // NOLINT
    if (ret < 0) {
        logger_->error("URG Open failed, error: {}", ret);
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(initializationWaitInterval));
    initialized_ = true;
    if (!setConfiguration()) {
        logger_->error("Failed to configure");
        return false;
    }
    logger_->info("Hokuyo initialized");
    return true;
}

bool HokuyoLaser::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return true;
    }
    urg_close(&urg_);
    initialized_ = false;
    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr HokuyoLaser::getPointCloud() {
    logger_->debug("getPointCloud");
    std::vector<std::array<float, 2>> laserScan = getScan();
    if (laserScan.empty()) {
        logger_->error("Could not obtain new laser scan");
        return nullptr;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->is_dense = true;
    cloud->width = laserScan.size();
    cloud->height = 1;
    cloud->points.resize(laserScan.size());
    // TODO(jukabala): cloud->sensor_origin_ to be added from position manager
    // TODO(jukabala): cloud->sensor_orientation_ to be added from position manager
    for (unsigned int i = 0; i < laserScan.size(); i++) {
        pcl::PointXYZ point = pcl::PointXYZ();
        cloud->points[i] = point;
        auto xyPoint = utility::visionutility::pointcloud::pointconverter::spherical2cartesian(
            laserScan[i][0], laserScan[i][1]);
        cloud->points[i].x = xyPoint[0];
        cloud->points[i].y = xyPoint[1];
        cloud->points[i].z = xyPoint[2];
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr HokuyoLaser::getTargetPointCloud() {
    logger_->debug("getPointCloud");
    std::vector<std::array<float, 2>> laserScan = getScan();
    if (laserScan.empty()) {
        logger_->error("Could not obtain new laser scan");
        return nullptr;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->is_dense = true;
    cloud->width = laserScan.size();
    cloud->height = 1;
    cloud->points.resize(laserScan.size());
    // TODO(jukabala): cloud->sensor_origin_ to be added from position manager
    // TODO(jukabala): cloud->sensor_orientation_ to be added from position manager
    for (unsigned int i = 0; i < laserScan.size(); i++) {
        pcl::PointXYZ point = pcl::PointXYZ();
        cloud->points[i] = point;
        auto xyPoint = utility::visionutility::pointcloud::pointconverter::spherical2cartesian(
            laserScan[i][0], laserScan[i][1]);
        if (xyPoint[0] >= laserConfig_->getFrameLimits().x_min &&
            xyPoint[0] <= laserConfig_->getFrameLimits().x_max &&
            xyPoint[2] >= laserConfig_->getFrameLimits().y_min &&
            xyPoint[2] <= laserConfig_->getFrameLimits().y_max &&
            xyPoint[1] >= laserConfig_->getFrameLimits().z_min &&
            xyPoint[1] <= laserConfig_->getFrameLimits().z_max) {
            cloud->points[i].x = xyPoint[0];
            cloud->points[i].y = xyPoint[1];
            cloud->points[i].z = xyPoint[2];
        }
    }
    return cloud;
}

std::shared_ptr<LaserConfiguration> HokuyoLaser::getConfiguration() {
    return laserConfig_;
}

std::vector<std::array<float, 2>> HokuyoLaser::getScan() {
    logger_->debug("getScan");
    std::vector<std::array<float, 2>> measurement;
    if (!initialized_) {
        return measurement;
    }
    int scanTimes = 1;
    int skipScan = 0;
    int result = urg_start_measurement(&urg_, URG_DISTANCE, scanTimes, skipScan);
    if (result < 0) {
        logger_->error("Start measurement failed, error: {}", result);
        return measurement;
    }
    long int timestamp;  // NOLINT
    long int* data = new long int[urg_max_data_size(&urg_)];  // NOLINT
    int retCode = urg_get_distance(&urg_, data,  &timestamp);
    for (int j = 0; j < retCode; j++) {
        float range = (static_cast<float>(data[j]))/1000;
        if (range < laserConfig_->getLaserParameters().minRange ||
            range > laserConfig_->getLaserParameters().maxRange) {
            continue;
        }
        float theta = urg_index2rad(&urg_, j);
        measurement.push_back({range, theta});
    }
    delete[] data;
    return measurement;
}

bool HokuyoLaser::setConfiguration() {
    logger_->debug("setConfiguration");
    auto scanIntervalUS = std::chrono::microseconds(urg_scan_usec(&urg_));
    if (!laserConfig_->setScanIntervalUS(scanIntervalUS)) {
        return false;
    }
    LaserParameters parameters;
    long int minDistance = 0;  // NOLINT
    long int maxDistance = 0;  // NOLINT
    urg_distance_min_max(&urg_, &minDistance, &maxDistance);
    parameters.minRange = static_cast<float>(minDistance*1e-3);
    parameters.maxRange = static_cast<float>(maxDistance*1e-3);
    int minStep = 0;
    int maxStep = 0;
    urg_step_min_max(&urg_, &minStep, &maxStep);
    parameters.minAngle = static_cast<float>(urg_step2deg(&urg_, minStep));
    parameters.maxAngle = static_cast<float>(urg_step2deg(&urg_, maxStep));
    parameters.scanSize = urg_max_data_size(&urg_);
    if (!laserConfig_->setLaserParameters(parameters)) {
        return false;
    }
    return true;
}

}  // namespace laser
}  // namespace sensors
}  // namespace crf
