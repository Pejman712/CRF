#pragma once

/* © Copyright CERN 2020.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
*/

#include <string>
#include <map>
#include <boost/optional.hpp>
#include <atomic>
#include <memory>

#include "EventLogger/EventLogger.hpp"
#include "RobotBase/IEtherCATRobotBase.hpp"

using crf::devices::ethercatdevices::ISoemApi;

namespace crf {
namespace actuators {
namespace robotbase {

class EtherCATRobotBase : public IEtherCATRobotBase {
 public:
    EtherCATRobotBase() = delete;
    EtherCATRobotBase(const EtherCATRobotBase&) = delete;
    EtherCATRobotBase(EtherCATRobotBase&&) = delete;
    explicit EtherCATRobotBase(const std::string& ifname, int IOMapSize = 4096,
    std::shared_ptr<ISoemApi> soemApi = nullptr);
    ~EtherCATRobotBase() override;
    bool initialize() override;
    bool deinitialize() override;

    boost::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
    getMotorFrontLeft() override;

    boost::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
    getMotorFrontRight() override;

    boost::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
    getMotorBackLeft() override;

    boost::optional<std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>>
    getMotorBackRight() override;

    boost::optional<std::shared_ptr<devices::ethercatdevices::EtherCATManager>>
    getManager() override;

 private:
    utility::logger::EventLogger logger_;
    std::string portName_;
    std::atomic<bool> initialized_;
    std::shared_ptr<ISoemApi> soemApi_;
    int IOMapSize_;
    std::shared_ptr<devices::ethercatdevices::EtherCATManager> ECManager_;
    std::map<int, std::shared_ptr<devices::ethercatdevices::IEtherCATMotor>> motorsMap_;
};

}  // namespace robotbase
}  // namespace robots
}  // namespace cern
