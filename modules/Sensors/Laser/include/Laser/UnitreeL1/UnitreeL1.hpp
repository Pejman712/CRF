/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Dadi Hrannar Davidsson CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>


#include <unitree_lidar_sdk.h> 


#include "EventLogger/EventLogger.hpp"
#include "Laser/ILaser.hpp"
#include "Laser/Point3D.hpp"

namespace crf {
namespace sensors {
namespace laser {

/**
 * 
 */
class UnitreeL1: public ILaser {
 public:
    UnitreeL1(LaserConnectionType connectionType, const std::string& deviceOrAdress,
        unsigned int baudrateOrPort, const nlohmann::json& laserJSON);
    UnitreeL1() = delete;
    UnitreeL1(const UnitreeL1&) = delete;
    UnitreeL1(UnitreeL1&&) = delete;

    ~UnitreeL1() override;

    bool initialize() override;
    bool deinitialize() override;
    pcl::PointCloud<Point3D>::Ptr getPointCloud() override;
    pcl::PointCloud<Point3D>::Ptr getTargetPointCloud() override;
    std::shared_ptr<LaserConfiguration> getConfiguration() override;
    bool checkConnection();
 private:
    nlohmann::json laserJSON_;
    std::shared_ptr<LaserConfiguration> laserConfig_;
    LaserConnectionType connectionType_;
    std::string deviceOrAdress_;
    unsigned int baudrateOrPort_;
    crf::utility::logger::EventLogger logger_;
    bool initialized_;
    std::unique_ptr<unitree_lidar_sdk::UnitreeLidarReader> lreader_;
   

    /**
     * @brief Get the Scan object
     * 
     * @return std::vector<std::array<float, 2>> Vector of arrays where the 0th element is the
     *         distance in meters and the 1st the angle in radians
     */
    //std::vector<std::array<float, 2>> getScan();
    //std::vector<std::array<float, 2>> getDefinedScan();
};

 void transformUnitreeCloudToPCL(const unitree_lidar_sdk::PointCloudUnitree& cloudIn, pcl::PointCloud<Point3D>::Ptr cloudOut);


}  // namespace laser
}  // namespace sensors
}  // namespace crf
