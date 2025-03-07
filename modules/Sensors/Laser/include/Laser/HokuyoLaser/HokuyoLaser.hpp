#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <memory>
#include <string>
#include <vector>

#include <urg_c/urg_connection.h>
#include <urg_c/urg_sensor.h>

#include "EventLogger/EventLogger.hpp"
#include "Laser/ILaser.hpp"

#define INIT_WAIT_INTERVAL_MS 250

namespace crf {
namespace sensors {
namespace laser {

enum HokuyoConnectionType {
    SERIAL = 0,
    ETHERNET = 1
};

struct HokuyoDataPoint {
    float range;  // distance in [m]
    float theta;  // angle in [rad]
};

/*
 * Class for communication with Hukoyo Laser Scanner Family, validated on
 * e.g. UST20LX, URG04LXUG01
 */
class HokuyoLaser: public ILaser {
 public:
    HokuyoLaser() = delete;
    HokuyoLaser(const HokuyoLaser&) = delete;
    HokuyoLaser(HokuyoLaser&&) = delete;
    HokuyoLaser(HokuyoConnectionType connectionType,
        const std::string& deviceOrAdress, const int& baudrateOrPort);
    ~HokuyoLaser() override;

    bool initialize() override;
    bool deinitialize() override;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointCloud() override;
    std::shared_ptr<LaserConfiguration> getConfiguration() override;

 private:
    std::shared_ptr<LaserConfiguration> laserConfig_;
    HokuyoConnectionType connectionType_;
    std::string deviceOrAdress_;
    int baudrateOrPort_;
    bool initialized_;
    utility::logger::EventLogger logger_;
    //! URG Lidar descriptor.
    urg_t urg_;
    std::vector<HokuyoDataPoint> getScan();
    bool setConfiguration();
};

}  // namespace laser
}  // namespace sensors
}  // namespace crf
