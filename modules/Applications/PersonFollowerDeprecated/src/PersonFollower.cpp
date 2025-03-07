/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Arturs Ivanovs CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include "Types/TaskTypes/TaskVelocity.hpp"
#include "Types/TaskTypes/TaskPose.hpp"
#include "PersonFollower/PersonFollower.hpp"
#include "LaserCommunicationPoint/LaserPacket.hpp"

#define SET_DISTANCE_TO_PERSON 5.0f
#define PERSONFOLLOWER_PROPORTIONAL_COEFF 0.2f
#define WALLFOLLOWER_PROPORTIONAL_COEFF 0.05f

using crf::applications::robotbasecontroller::IRobotBaseController;
using crf::utility::types::TaskVelocity;

namespace crf {
namespace applications {
namespace personfollower {

PersonFollower::PersonFollower(std::shared_ptr<IPersonTracker> tracker,
    std::shared_ptr<IRobotBaseController> baseController,
    crf::utility::types::TaskPose cameraPose,
    std::shared_ptr<IPC> personTrackerOutputIpc,
    std::shared_ptr<walldetector::IWallDetector> wallDetector):
    tracker_(tracker),
    baseController_(baseController),
    cameraPose_(cameraPose),
    personTrackerOutputIpc_(personTrackerOutputIpc),
    wallDetector_(wallDetector),
    calibrated_(false),
    stopSignal_(false),
    logger_("PersonFollower") {
    logger_->debug("CTor");
}

PersonFollower::~PersonFollower() {
    logger_->debug("DTor");
}

bool PersonFollower::initialize() {
    logger_->debug("initialize");
    if (app_.joinable()) {
        logger_->warn("PersonFollower has been already initialized before");
        return false;
    }
    if (!tracker_->initialize()) {
        logger_->error("tracker could not be initialized");
        return false;
    }
    if (!personTrackerOutputIpc_->open()) {
        logger_->error("IPC reader could not be opened");
        return false;
    }
    if (!baseController_->initialize()) {
        logger_->error("robotBaseController could not be initialized");
        return false;
    }
    try {
        app_ = std::thread(&PersonFollower::followPerson, this);
    } catch (const std::system_error& e) {
        logger_->error("There is a problem to initialize the personfollower thread: {0}", e.what());
        return false;
    }
    return true;
}

bool PersonFollower::deinitialize() {
    logger_->debug("deinitialize");
    if (!app_.joinable()) {
        logger_->warn("PersonFollower has not been initialized, so it can't be deinitialize");
        return false;
    }
    stopSignal_ = true;
    try {
        app_.join();
    } catch (const std::system_error& e) {
        logger_->error(
          "There is a problem to join the personfollower thread to the main: {0}", e.what());
        return false;
    }
    personTrackerOutputIpc_->close();
    logger_->debug("The PersonFollower has been stopped");
    return true;
}

void PersonFollower::followPerson() {
    while (!stopSignal_) {
        logger_->debug("followPerson");
        TaskVelocity targetVelocity = {};
        if (!calibrated_) {
            logger_->info("calibrating...");
            int displacement = tracker_->getCameraDisplacement();
            if ((displacement < 20) && (displacement > -20)) {
                calibrated_ = true;
                logger_->debug("Camera adjusted");
            } else if (displacement == 666) {
                logger_->warn("Person is not detected in camera frame");
            } else {
              logger_->warn("Camera displacement: {}", displacement);
              if (displacement > 0) {
                  targetVelocity(5) = .05;
              } else if (displacement < 0) {
                  targetVelocity(5) = -.05;
              }
            }
        } else {
            logger_->info("camera is calibrated");
            PersonCentroid personPoint = tracker_->trackPerson(cameraPose_);
            if (personPoint.objectID == -1) {
                logger_->warn("No person found.");
                calibrated_ = false;
                continue;
            } else {
                targetVelocity(0) = -(personPoint.euclideanDistance-SET_DISTANCE_TO_PERSON)*
                    PERSONFOLLOWER_PROPORTIONAL_COEFF;
            }
            logger_->warn("PersonDistance: {} m, objectID: {}, dissapeared: {}",
              personPoint.euclideanDistance, personPoint.objectID, personPoint.dissapeared);
            // THIS PART IS A SIMPLE INTEGRATION OF WALLFOLLOWER
            Packets::PacketHeader header;
            Packets::LaserPacket laserPacket;
            std::string buf;
            personTrackerOutputIpc_->read(buf, header);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud
            (new pcl::PointCloud<pcl::PointXYZRGBA>());
            if (header.type() != Packets::LASER_PACKET_TYPE) {
                logger_->warn("Wrong packet type!");
                pointCloud = nullptr;
            }
            laserPacket.deserialize(buf);
            if (laserPacket.pointCloud.points.size() == 0) {
                logger_->warn("Got empty data");
                pointCloud = nullptr;
            }
            for (size_t i = 0; i < laserPacket.pointCloud.points.size(); i++) {
                pointCloud->points.push_back(laserPacket.pointCloud.points[i]);
            }
            auto wallVector = wallDetector_->detectWall(pointCloud, cameraPose_);
            if (wallVector.empty()) {
                logger_->warn("No wall detected");
            } else {
                targetVelocity(5) += (-wallVector[0].theta*WALLFOLLOWER_PROPORTIONAL_COEFF);
                  logger_->info("rotating to adjust for the wall slope");
            }
            // END OF WALLFOLLOWER CODE
        }
        baseController_->setVelocity(targetVelocity);
        auto controllerVelocityValid = baseController_->getVelocity();
        if (!controllerVelocityValid) {
             logger_->warn("A problem occured while getting robot velocity");
        } else {
            logger_->info("robotVelocity: {}", controllerVelocityValid.get());
        }
    }
}


}  // namespace personfollower
}  // namespace applications
}  // namespace crf
