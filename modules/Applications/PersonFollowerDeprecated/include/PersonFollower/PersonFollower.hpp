#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <thread>
#include <iostream>

#include "EventLogger/EventLogger.hpp"
#include "PersonFollower/IPersonTracker.hpp"
#include "RobotBaseControllers/IRobotBaseController.hpp"
#include "PersonFollower/IPersonFollower.hpp"
#include "WallDetector/IWallDetector.hpp"
#include "IPC/IPC.hpp"
#include "LaserCommunicationPoint/LaserCommunicationPoint.hpp"

namespace crf {
namespace applications {
namespace personfollower {

class PersonFollower: public IPersonFollower {
 public:
    PersonFollower(std::shared_ptr<IPersonTracker> tracker,
      std::shared_ptr<crf::applications::robotbasecontroller::IRobotBaseController> baseController,
      crf::utility::types::TaskPose cameraPose,
      std::shared_ptr<IPC> personTrackerOutputIpc,
      std::shared_ptr<walldetector::IWallDetector> wallDetector);
    PersonFollower(const PersonFollower& other) = delete;
    PersonFollower(PersonFollower&& other) = delete;
    PersonFollower() = delete;
    ~PersonFollower() override;
    bool initialize() override;
    bool deinitialize() override;

 private:
    void followPerson();
    std::shared_ptr<IPersonTracker> tracker_;
    std::shared_ptr<crf::applications::robotbasecontroller::IRobotBaseController> baseController_;
    crf::utility::types::TaskPose cameraPose_;
    std::shared_ptr<IPC> personTrackerOutputIpc_;
    std::shared_ptr<walldetector::IWallDetector> wallDetector_;
    bool calibrated_;
    bool stopSignal_;
    std::thread app_;
    utility::logger::EventLogger logger_;
};

}  // namespace personfollower
}  // namespace applications
}  // namespace crf
