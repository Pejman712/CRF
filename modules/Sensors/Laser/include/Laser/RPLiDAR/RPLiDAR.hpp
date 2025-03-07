/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *         David Forkel CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rplidar.h>

#include "EventLogger/EventLogger.hpp"
#include "Laser/ILaser.hpp"
#include "Laser/Point3D.hpp"

namespace crf {
namespace sensors {
namespace laser {

/**
 * 
 */
class RPLiDAR: public ILaser {
 public:
    RPLiDAR(LaserConnectionType connectionType, const std::string& deviceOrAdress,
        unsigned int baudrateOrPort, const nlohmann::json& laserJSON);
    RPLiDAR() = delete;
    RPLiDAR(const RPLiDAR&) = delete;
    RPLiDAR(RPLiDAR&&) = delete;

    ~RPLiDAR() override;

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
    sl::ILidarDriver* driver_;
    sl::IChannel* channel_;

    /**
     * @brief Get the Scan object
     * 
     * @return std::vector<std::array<float, 2>> Vector of arrays where the 0th element is the
     *         distance in meters and the 1st the angle in radians
     */
    std::vector<std::array<float, 2>> getScan();
    std::vector<std::array<float, 2>> getDefinedScan(); 
};


}  // namespace laser
}  // namespace sensors
}  // namespace crf
