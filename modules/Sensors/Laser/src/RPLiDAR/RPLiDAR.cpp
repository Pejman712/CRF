/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *         David Forkel CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <rplidar.h>
#include <libusb.h>  // install libusb + libudev

#include "Laser/RPLiDAR/RPLiDAR.hpp"
#include "VisionUtility/PointCloud/PointConverter.hpp"
#include "Laser/Point3D.hpp"

namespace crf {
namespace sensors {
namespace laser {

RPLiDAR::RPLiDAR(LaserConnectionType connectionType, const std::string& deviceOrAdress,
    unsigned int baudrateOrPort, const nlohmann::json& laserJSON) :
    laserJSON_(laserJSON),
    laserConfig_(new LaserConfiguration()),
    connectionType_(connectionType),
    deviceOrAdress_(deviceOrAdress),
    baudrateOrPort_(baudrateOrPort),
    logger_("RPLiDAR"),
    initialized_(),
    driver_(),
    channel_() {
    logger_->debug("CTor");
    if (!laserConfig_->parse(laserJSON_)) {
        throw std::runtime_error("Failed to read the configuration file");
    }
}

RPLiDAR::~RPLiDAR() {
    logger_->debug("DTor");
    deinitialize();
    delete driver_;
    delete channel_;
}

bool RPLiDAR::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->debug("Already initialized");
        return true;
    }
    sl::Result<sl::ILidarDriver*> laserCreation = sl::createLidarDriver();
    if (!SL_IS_OK(laserCreation.err)) {
        logger_->error("Insufficent memory");
        return false;
    }
    driver_ = laserCreation.value;
    if (connectionType_ == LaserConnectionType::Ethernet) {
        sl::Result<sl::IChannel*> channelCreation = sl::createUdpChannel(deviceOrAdress_,
            baudrateOrPort_);
        if (!SL_IS_OK(channelCreation.err)) {
            logger_->error("Failed to create ethernet UDP channel");
            return false;
        }
        channel_ = channelCreation.value;
    } else if (connectionType_ == LaserConnectionType::Serial) {
        sl::Result<sl::IChannel*> channelCreation = sl::createSerialPortChannel(deviceOrAdress_,
            baudrateOrPort_);
        if (!SL_IS_OK(channelCreation.err)) {
            logger_->error("Failed to create serial channel");
            return false;
        }
        channel_ = channelCreation.value;
    } else {
        logger_->error("Connection type unsupported");
        return false;
    }
    if (!SL_IS_OK(driver_->connect(channel_))) {
        logger_->error("Failed to connect");
        return false;
    }
    sl_lidar_response_device_info_t info;
    if (!SL_IS_OK(driver_->getDeviceInfo(info))) {
        logger_->error("Failed to retrive device information");
        return false;
    }
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ; ++pos) {
        printf("%02X", info.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , info.firmware_version>>8
            , info.firmware_version & 0xFF
            , static_cast<int>(info.hardware_version));
    
    /* Temporary solution to print S/N, Firmware-, Hardware-version
    logger_->info("Connected to {} - SN {}", info.model, info.serialnum);
    logger_->info("RPLiDAR Hardware Version {}", info.hardware_version);
    logger_->info("RPLiDAR Firmware Version {}.{}", info.firmware_version >> 8,
        info.firmware_version & 0xFF);*/

    sl_lidar_response_device_health_t health;
    if (!SL_IS_OK(driver_->getHealth(health))) {
        logger_->error("Failed to retrive device status");
        return false;
    }
    if (health.status == SL_LIDAR_STATUS_WARNING) {
        logger_->warn("Status - {}", static_cast<int>(health.error_code));
    } else if (health.status == SL_LIDAR_STATUS_ERROR) {
        logger_->error("Status - {} - Rebooting device", static_cast<int>(health.error_code));
        driver_->reset();
        return false;
    }

    if (!SL_IS_OK(driver_->startScan(false, true))) {
        logger_->error("Failed to start the scan");
        return false;
    }

    /* Freqeunacy is defined based on the linear equation estimated from LiDAR adquisition time and the RPM of the RPLiDAR*/

    if (!SL_IS_OK((driver_->setMotorSpeed((1.9 +laserConfig_->getLaserParameters().frequency)/0.0245)))) {
        logger_->error("Failed to set motor speed");
        return false;
    }
    initialized_ = true;
    return true;
}

bool RPLiDAR::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return true;
    }
    if (!SL_IS_OK(driver_->stop())) {
       logger_->error("Failed to stop the scanning");
        return false;
    }
    if (!SL_IS_OK(driver_->setMotorSpeed(0))) {
        logger_->error("Failed to stop the motor");
        return false;
    }
    initialized_ = false;
    return true;
}

pcl::PointCloud<Point3D>::Ptr RPLiDAR::getPointCloud() {
    logger_->debug("getPointCloud");
    std::vector<std::array<float, 2>> laserScan = getScan();
    if (laserScan.empty()) {
        logger_->error("Could not obtain new laser scan");
        return nullptr;
    }
    pcl::PointCloud<Point3D>::Ptr cloud(new pcl::PointCloud<Point3D>);
    cloud->is_dense = true;
    cloud->width = laserScan.size();
    cloud->height = 1;
    cloud->points.resize(laserScan.size());
    for (unsigned int i = 0; i < laserScan.size(); i++) {
        Point3D  point = Point3D();
        cloud->points[i] = point;
        auto xyPoint = utility::visionutility::pointcloud::pointconverter::spherical2cartesian(
            laserScan[i][0], laserScan[i][1], 0);
        cloud->points[i].x = xyPoint[0];
        cloud->points[i].y = xyPoint[2];
        cloud->points[i].z = xyPoint[1];
        cloud->points[i].intensity = 0.f;
        cloud->points[i].ring = 0;
        cloud->points[i].time = 0;
    }
    return cloud;
}

pcl::PointCloud<Point3D>::Ptr RPLiDAR::getTargetPointCloud() {
    logger_->debug("getTargetPointCloud");
    std::vector<std::array<float, 2>> laserScan = getDefinedScan();
    if (laserScan.empty()) {
        logger_->error("Could not obtain new laser scan");
        return nullptr;
    }
    pcl::PointCloud<Point3D>::Ptr cloud(new pcl::PointCloud<Point3D>);
    cloud->is_dense = true;
    cloud->width = laserScan.size();
    cloud->height = 1;
    cloud->points.resize(laserScan.size());
    for (unsigned int i = 0; i < laserScan.size(); i++) {
        Point3D point = Point3D();
        cloud->points[i] = point;
        auto xyPoint = utility::visionutility::pointcloud::pointconverter::spherical2cartesian(
            laserScan[i][0], laserScan[i][1], 0);
        if (xyPoint[0] >= laserConfig_->getFrameLimits().x_min &&
            xyPoint[0] <= laserConfig_->getFrameLimits().x_max &&
            xyPoint[2] >= laserConfig_->getFrameLimits().y_min &&
            xyPoint[2] <= laserConfig_->getFrameLimits().y_max &&
            xyPoint[1] >= laserConfig_->getFrameLimits().z_min &&
            xyPoint[1] <= laserConfig_->getFrameLimits().z_max) {
            cloud->points[i].x = xyPoint[0];
            cloud->points[i].y = xyPoint[2];
            cloud->points[i].z = xyPoint[1];
            cloud->points[i].intensity = 0.f;
            cloud->points[i].ring =  0;
            cloud->points[i].time = 0.f;

        }
       
    }

    return cloud;
}

std::shared_ptr<LaserConfiguration> RPLiDAR::getConfiguration() {
    logger_->debug("getConfiguration");
    return laserConfig_;
}
bool RPLiDAR::checkConnection()  {
    libusb_device **devs;  // pointer to pointer of device, used to retrieve a list of devices
    libusb_context *ctx = NULL;  // libusb session
    int r;  // for return values
    ssize_t cnt;  // holding number of devices in list
    r = libusb_init(&ctx);  // initialize a library session
    if (r < 0) {
        logger_->error("Device List could not be created");
        return false;
    }
    cnt = libusb_get_device_list(ctx, &devs);  // get the list of devices
    if (cnt < 0) {
        logger_->error("Devices could not be fetched");
        return false;
    }
    ssize_t i;  // for iterating through the list
    libusb_device_descriptor desc;
    for (i = 0; i < cnt; i++) {
        int r = libusb_get_device_descriptor(devs[i], &desc);
        if (r < 0) {
            logger_->error("Failed to get device descriptor");
            return false;
        }
        if ((desc.idVendor == laserConfig_->getLaserParameters().LidarVendorID) &&
            (desc.idProduct == laserConfig_->getLaserParameters().LidarProductID)) {
            logger_->debug("RPLidar found");
            break;
        }

        if (i == (cnt-1)) {
            logger_->error("No LiDAR connected");
            return false;
        }
    }
    return true;
}

std::vector<std::array<float, 2>> RPLiDAR::getScan() {
    std::vector<std::array<float, 2>> output;
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    std::size_t count = sizeof(nodes) / sizeof(nodes[0]);
    if (!SL_IS_OK(driver_->grabScanDataHq(nodes, count))) {
        logger_->warn("Failed to retrieve points");
        return output;
    }
    driver_->ascendScanData(nodes, count);
    for (std::size_t point = 0; point < count ; ++point) {
        output.push_back({
            nodes[point].dist_mm_q2 / 1000.f / (1 << 2),
            (nodes[point].angle_z_q14 * 90.f / (1 << 14)) * static_cast<float>(M_PI/180)});
    }
    return output;
}




std::vector<std::array<float, 2>> RPLiDAR::getDefinedScan() {
    std::vector<float> distance;
    std::vector<float> angle;
    std::vector<std::array<float, 2>> output;
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    std::size_t count = sizeof(nodes) / sizeof(nodes[0]);
    if (!SL_IS_OK(driver_->grabScanDataHq(nodes, count))) {
        logger_->warn("Failed to retrieve points");
        return output;
    }
    driver_->ascendScanData(nodes, count);
    
    float theta = 3* M_PI / 2; 
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);

    for (std::size_t point = 0; point < count; ++point) {
        float dist = nodes[point].dist_mm_q2 / 1000.f / (1 << 2);
        float ang = nodes[point].angle_z_q14 * 90.f / (1 << 14) * static_cast<float>(M_PI / 180);

        distance.push_back(dist);
        angle.push_back(ang);

        if (dist >= laserConfig_->getLaserParameters().minRange &&
            dist <= laserConfig_->getLaserParameters().maxRange &&
            ang >= laserConfig_->getLaserParameters().minAngle &&
            ang <= laserConfig_->getLaserParameters().maxAngle) {
            
            // Convert polar coordinates to Cartesian coordinates
            float x = dist * cos(ang);
            float y = dist * sin(ang);
            
            // Apply rotation
            float x_rot = cos_theta * x - sin_theta * y;
            float y_rot = sin_theta * x + cos_theta * y;
            
            // Convert back to polar coordinates 
            float dist_rot = sqrt(x_rot * x_rot + y_rot * y_rot);
            float ang_rot = atan2(y_rot, x_rot);

            // Store the rotated coordinates
            output.push_back({dist_rot, ang_rot});
        }
    }
    
    return output;
}


}  // namespace laser
}  // namespace sensors
}  // namespace crf
