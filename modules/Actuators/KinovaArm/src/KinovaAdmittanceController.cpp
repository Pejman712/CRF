/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#define NUMBER_OF_JOINTS 6

#include "KinovaArm/KinovaAdmittanceController.hpp"
#include "Types/Types.hpp"
#include "KinovaArm/KinovaJaco.hpp"
#include "RobotArmKinematics/IRobotArmKinematics.hpp"

#include <string>
#include <fstream>
#include <future>
#include <memory>
#include <cmath>
#include <chrono>
#include <thread>

#include <nlohmann/json.hpp>

namespace crf::actuators::kinovaarm {

KinovaAdmittanceController::KinovaAdmittanceController(
        std::shared_ptr<KinovaJaco> arm,
        std::shared_ptr<crf::control::robotarmkinematics::IRobotArmKinematics> kinematics,
        const std::string& configFilePath) :
        logger_("KinovaAdmittanceController"),
        arm_(arm),
        kinematics_(kinematics),
        armConfiguration_(arm_->getConfiguration()),
        configFile_(configFilePath),
        jointLimits_(NUMBER_OF_JOINTS),
        torqueMinLimits_(NUMBER_OF_JOINTS),
        torqueMaxLimits_(NUMBER_OF_JOINTS),
        velocityLowPassFilter_(NUMBER_OF_JOINTS),
        currentJointPositions_(NUMBER_OF_JOINTS),
        currentJointVelocities_(NUMBER_OF_JOINTS),
        currentJointForceTorques_(NUMBER_OF_JOINTS),
        rtLoopTime_(std::chrono::duration_cast<std::chrono::microseconds>(
                armConfiguration_->getRTLoopTime())),
        controlLoopThread_(),
        runControlLoop_(false),
        positionControl_(false),
        targetJointPositions_(6) {
    logger_->debug("CTor");
}

KinovaAdmittanceController::~KinovaAdmittanceController() {
    deinitialize();
    logger_->debug("dTor()");
}

bool KinovaAdmittanceController::initialize() {
    if (controlLoopThread_.joinable()) {
        logger_->debug("KinovaAdmittanceController already initialized");
        return false;
    }

    // read configfile
    std::ifstream admittanceData(configFile_);
    if ((admittanceData.rdstate() & std::ifstream::failbit) != 0) {
        logger_->error("There is some problem with the config file");
        return false;
    }

    nlohmann::json robotJSON;
    try {
        admittanceData >> robotJSON;

        for (int i=0; i < NUMBER_OF_JOINTS; i++) {
            torqueMinLimits_[i] =
                    robotJSON.at("TorqueLimits").at("MinimumTorque")[i].get<float>();
            torqueMaxLimits_[i] =
                    robotJSON.at("TorqueLimits").at("MaximumTorque")[i].get<float>();
            jointLimits_[i] = robotJSON.at("JointLimitsDeg")[i].get<float>() / 180.0 * M_PI;
            velocityLowPassFilter_.at(i) = robotJSON.at("VelocityLowPassFilter")[i].get<float>();
        }
    } catch (const std::exception& e) {
        logger_->error("Failed to parse config file because: {}", e.what());
        return false;
    }
    logger_->debug("Config file parsed!");

    // gohome, zerotorque
    if (!arm_->moveHomePosition()) {
        logger_->error("Could not move home!");
        return false;
    }
    if (!arm_->setJointPositions(utility::types::JointPositions(6))) {
        logger_->error("Move to vertical position failed!");
        return false;
    }
    bool arrived = false;
    while (!arrived) {
        currentJointPositions_ = arm_->getJointPositions().get();
        arrived = utility::types::areAlmostEqual(currentJointPositions_,
                                        utility::types::JointPositions(6));
    }
    // Wait for the arm to settle down
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    arm_->zeroJointForceTorques();

    // start thread
    startControlLoop();

    logger_->debug("Initialized");
    return true;
}

bool KinovaAdmittanceController::deinitialize() {
    if (!controlLoopThread_.joinable()) {
        logger_->debug("KinovaAdmittanceController already deinitialized");
        return false;
    }
    stopControlLoop();
    logger_->debug("Deinitialized");
    return true;
}

bool KinovaAdmittanceController::setJointPositions(
    const utility::types::JointPositions& jointPositions) {
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
        if (std::fabs(jointPositions[i]) > jointLimits_[i]) {
            return false;
        }
    }
    if (arm_->setJointPositions(jointPositions)) {
        targetJointPositions_ = jointPositions;
        positionControl_ = true;
        return true;
    } else {
        return false;
    }
}

utility::types::JointPositions KinovaAdmittanceController::getJointPositions() {
    return currentJointPositions_;
}

utility::types::TaskPose KinovaAdmittanceController::getTaskPose() {
    boost::optional<utility::types::TaskPose> cp =
            kinematics_->getPositionForwardKinematic(currentJointPositions_);
    return cp.get();
}

bool KinovaAdmittanceController::startControlLoop() {
    logger_->debug("startControlLoop()");
    if (controlLoopThread_.joinable()) {
        logger_->warn("mainControlLoop already started");
        return false;
    }
    runControlLoop_ = true;
    controlLoopThread_ = std::thread([this]() {
        controlLoop();
    });
    return true;
}

bool KinovaAdmittanceController::stopControlLoop() {
    logger_->debug("stopControlLoop()");
    if (!controlLoopThread_.joinable()) {
        logger_->warn("mainControlLoop was not started");
        return false;
    }
    runControlLoop_ = false;
    controlLoopThread_.join();
    return true;
}

void KinovaAdmittanceController::controlLoop() {
    auto appliedVelocity = utility::types::JointVelocities(6);
    while (runControlLoop_) {
        auto start = std::chrono::high_resolution_clock::now();

        currentJointPositions_ = arm_->getJointPositions().get();
        if (positionControl_) {
            if (utility::types::areAlmostEqual(currentJointPositions_, targetJointPositions_)) {
                positionControl_ = false;
            }
        } else {
            currentJointForceTorques_ = arm_->getJointForceTorques().get();

            auto calculatedVelocity = utility::types::JointVelocities(6);

            for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
                // Check if we are out of limits, only allow to move back
                float torqueDirection = std::copysign(1.0, currentJointForceTorques_[i]);
                if ((std::fabs(currentJointPositions_[i]) > jointLimits_[i]) &&
                    (torqueDirection * currentJointPositions_[i] < 0)) {
                    appliedVelocity[i] = 0;
                } else if (std::fabs(currentJointForceTorques_[i]) > torqueMinLimits_[i]) {
                    if (std::fabs(currentJointForceTorques_[i]) < torqueMaxLimits_[i]) {
                        calculatedVelocity[i] = -currentJointForceTorques_[i] /
                            torqueMaxLimits_[i] *
                            armConfiguration_->getJointsConfiguration()[i].maximumVelocity;
                    } else {
                        calculatedVelocity[i] = -std::copysign(1.0, currentJointForceTorques_[i]) *
                                    armConfiguration_->getJointsConfiguration()[i].maximumVelocity;
                    }
                }

                // Apply low pass filter, to reduce the vibration in movement
                // this is a simplified g-h filter, where h is 0
                appliedVelocity[i] = velocityLowPassFilter_.at(i) * appliedVelocity[i] +
                                     (1 - velocityLowPassFilter_.at(i)) * calculatedVelocity[i];
            }
            arm_->setJointVelocities(appliedVelocity);
            currentJointVelocities_ = appliedVelocity;
        }

        std::chrono::microseconds elapsed
                = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::high_resolution_clock::now() - start);
        if ((rtLoopTime_ - elapsed).count() > 0) {
            std::this_thread::sleep_for(rtLoopTime_ - elapsed);
        }
    }
}

}  // namespace crf::actuators::kinovaarm
