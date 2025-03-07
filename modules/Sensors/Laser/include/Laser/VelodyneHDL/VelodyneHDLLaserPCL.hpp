/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero & Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <memory>
#include <mutex>

#include <nlohmann/json.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/hdl_grabber.h>
#include <boost/thread/mutex.hpp>
#include <boost/signals2/connection.hpp>

#include "Laser/ILaser.hpp"
#include "Laser/VelodyneHDL/VelodyneHDLLaserConfiguration.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace laser {

/*
 * Class for communication with Velodyne HDL using the PCL grabber
 */
class VelodyneHDLLaserPCL: public laser::ILaser {
 public:
    VelodyneHDLLaserPCL() = delete;
    VelodyneHDLLaserPCL(const VelodyneHDLLaserPCL&) = delete;
    VelodyneHDLLaserPCL(VelodyneHDLLaserPCL&&) = delete;
    explicit VelodyneHDLLaserPCL(const std::string&) = delete;
    explicit VelodyneHDLLaserPCL(const nlohmann::json& laserJSON);
    ~VelodyneHDLLaserPCL() override;

    bool initialize() override;
    bool deinitialize() override;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointCloud() override;
    std::shared_ptr<crf::sensors::laser::LaserConfiguration> getConfiguration() override;

 private:
    void cloudCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);

    nlohmann::json laserJSON_;
    utility::logger::EventLogger logger_;
    bool isInitialized_;
    static bool readingPointCloud_;

    std::shared_ptr<VelodyneHDLLaserConfiguration> configuration_;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_;

    std::shared_ptr<pcl::HDLGrabber> grabber_;
    std::mutex cloudMutex_;
    boost::signals2::connection cloudConnection_;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getInternalPointCloud();
    static void timerFunction();
};

}  // namespace laser
}  // namespace sensors
}  // namespace crf
