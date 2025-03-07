/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Dadi Hrannar Davidsson CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */


#include <unitree_lidar_sdk.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl/register_point_struct.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/approximate_voxel_grid.hpp>
#include <pcl/filters/impl/radius_outlier_removal.hpp>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/impl/passthrough.hpp>

#include <boost/preprocessor/control/if.hpp>
#include <boost/preprocessor/punctuation/comma_if.hpp>
#include <boost/mpl/aux_/na_spec.hpp>


#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "Laser/ILaser.hpp"
#include "Laser/UnitreeL1/UnitreeL1.hpp"
#include "VisionUtility/PointCloud/PointConverter.hpp"
#include "Laser/Point3D.hpp"





namespace crf {
namespace sensors {
namespace laser {



// Constructor
UnitreeL1::UnitreeL1(LaserConnectionType connectionType, const std::string& deviceOrAdress,
                     unsigned int baudrateOrPort, const nlohmann::json& laserJSON):
    laserJSON_(laserJSON),
    laserConfig_(new LaserConfiguration()),
    connectionType_(connectionType),
    deviceOrAdress_(deviceOrAdress),
    baudrateOrPort_(baudrateOrPort),
    logger_("UnitreeL1"),
    initialized_(false) {
    logger_->debug("Constructor called");
    // Parse laser configuration from JSON
    if (!laserConfig_->parse(laserJSON_)) {
        throw std::runtime_error("Failed to read the configuration file");
    }
}

// Destructor
UnitreeL1::~UnitreeL1() {
    logger_->debug("Destructor called");
    deinitialize();
}    

// Initialize the Lidar
bool UnitreeL1::initialize() {
    lreader_ = std::unique_ptr<unitree_lidar_sdk::UnitreeLidarReader>(unitree_lidar_sdk::createUnitreeLidarReader());
    logger_->debug("Initialize called");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    int cloud_scan_num = 18;  // Set the cloud scan number, adjust if necessary
    if (lreader_->initialize(cloud_scan_num, deviceOrAdress_)) {
        logger_->error("Failed to initialize Unitree Lidar");
        return false;
    } else {
        logger_->info("Lidar initialized successfully");
    }

    // Set Lidar to STANDBY, then to NORMAL mode
    logger_->info("Set Lidar working mode to: STANDBY");
    lreader_->setLidarWorkingMode(unitree_lidar_sdk::STANDBY);
    sleep(5);

    logger_->info("Set Lidar working mode to: NORMAL");
    lreader_->setLidarWorkingMode(unitree_lidar_sdk::NORMAL);
    sleep(2);
    
    //logger_->debug("runParse result first: {}", result);
    initialized_ = true;
    return true;
}

// Deinitialize the Lidar
bool UnitreeL1::deinitialize() {
    logger_->debug("Deinitialize called");
    if (!initialized_) {
        logger_->warn("Lidar not initialized");
        return false;
    }

    // Set the Lidar to STANDBY mode and stop data capture
    lreader_->setLidarWorkingMode(unitree_lidar_sdk::STANDBY);
    initialized_ = false;
    return true;
}



pcl::PointCloud<Point3D>::Ptr UnitreeL1::getPointCloud() {
    logger_->debug("Fetching PointCloud");
    unitree_lidar_sdk::MessageType result;

    // Poll for data
    while (true) {
        result = lreader_->runParse();  // Polling for data

        if (result == unitree_lidar_sdk::POINTCLOUD) {
            logger_->info("PointCloud data received!");

            // Retrieve the cloud
            auto cloudData = lreader_->getCloud();
            pcl::PointCloud<Point3D>::Ptr cloud(new pcl::PointCloud<Point3D>());

            // Convert the Unitree Lidar cloud to PCL format
            transformUnitreeCloudToPCL(cloudData, cloud);
            return cloud;
        }

        usleep(500);  // Sleep between polling
    }
}


pcl::PointCloud<Point3D>::Ptr UnitreeL1::getTargetPointCloud() {
    logger_->debug("Fetching Target PointCloud");

    // Poll for data with a retry mechanism
    unitree_lidar_sdk::MessageType result;
    while (true) {
        result = lreader_->runParse();
        if (result == unitree_lidar_sdk::POINTCLOUD) {
            break;
        }
       usleep(500); // Sleep between retries
    }

    if (result != unitree_lidar_sdk::POINTCLOUD) {
        logger_->error("Failed to retrieve PointCloud data after retries");
        return nullptr;
    }

    // Get the Unitree Lidar data as PointCloudUnitree
    const unitree_lidar_sdk::PointCloudUnitree& laserScan = lreader_->getCloud();
    if (laserScan.points.empty()) {
        logger_->error("No data in defined scan");
        return nullptr;
    }

    // Create a new PCL PointCloud object
    pcl::PointCloud<Point3D>::Ptr cloud(new pcl::PointCloud<Point3D>());
    cloud->is_dense = true;
    cloud->width = laserScan.points.size();
    cloud->height = 1;
    cloud->points.resize(laserScan.points.size());

    // Filter points based on frame limits
    for (unsigned int i = 0; i < laserScan.points.size(); i++) {
        const auto& point = laserScan.points[i];
        
        // Filter points based on frame limits
        if (point.x >= laserConfig_->getFrameLimits().x_min &&
            point.x <= laserConfig_->getFrameLimits().x_max &&
            point.y >= laserConfig_->getFrameLimits().y_min &&
            point.y <= laserConfig_->getFrameLimits().y_max &&
            point.z >= laserConfig_->getFrameLimits().z_min &&
            point.z <= laserConfig_->getFrameLimits().z_max) {
            
            cloud->points[i].x = point.x;
            cloud->points[i].y = point.y;
            cloud->points[i].z = point.z;
            cloud->points[i].intensity = point.intensity;
        }
    }

    logger_->info("Target PointCloud successfully retrieved");
    return cloud;
}




std::shared_ptr<LaserConfiguration> UnitreeL1::getConfiguration() {
    return laserConfig_;
}


void transformUnitreeCloudToPCL(const unitree_lidar_sdk::PointCloudUnitree& cloudIn, pcl::PointCloud<Point3D>::Ptr cloudOut) {
    cloudOut->clear(); // Clear the output cloud before filling it
    for (const auto& point : cloudIn.points) {
        Point3D p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        p.intensity = point.intensity; // Assuming the input point has intensity
        p.time = point.time; // Handle the time field here
        p.ring = point.ring;
        cloudOut->push_back(p); // Add point to PCL cloud
    }
}


}  // namespace laser
}  // namespace sensors
}  // namespace crf






















