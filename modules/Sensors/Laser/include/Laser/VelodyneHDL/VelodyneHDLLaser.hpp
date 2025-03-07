/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <memory>
#include <deque>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <future>
#include <assert.h>
#include <mutex>

#include "Laser/ILaser.hpp"
#include "Laser/VelodyneHDL/VelodyneHDLLaserConfiguration.hpp"
#include "Laser/VelodyneHDL/VelodyneHDLPacketDriver.hpp"
#include "Laser/VelodyneHDL/VelodyneHDLPacketDecoder.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace laser {

/*
 * Class for communication with Velodyne HDL without using the PCL grabber (HDL version)
 */
class VelodyneHDLLaser: public laser::ILaser {
 public:
    VelodyneHDLLaser() = delete;
    VelodyneHDLLaser(const VelodyneHDLLaser&) = delete;
    VelodyneHDLLaser(VelodyneHDLLaser&&) = delete;
    explicit VelodyneHDLLaser(const std::string&) = delete;
    explicit VelodyneHDLLaser(const nlohmann::json& laserJSON);
    ~VelodyneHDLLaser() override;

    bool initialize() override;
    bool deinitialize() override;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointCloud() override;
    std::shared_ptr<crf::sensors::laser::LaserConfiguration> getConfiguration() override;

 private:
    nlohmann::json laserJSON_;
    crf::utility::logger::EventLogger logger_;
    static bool isInitialized_;
    static bool readingPointCloud_;

    std::shared_ptr<VelodyneHDLLaserConfiguration> configuration_;
    VelodyneHDLLaserNetworkConfiguration networkConfig_;

    VelodyneHDLPacketDriver driver_;
    VelodyneHDLPacketDecoder decoder_;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getInternalPointCloud();
    static void timerFunction(std::future<void> futureObj);
};

}  // namespace laser
}  // namespace sensors
}  // namespace crf
