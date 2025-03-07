/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN BE/CEM/MRO 2022
 *         Hannes Gamper CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <bitset>
#include <cmath>
#include <exception>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <nlohmann/json.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include "crf/expected.hpp"
#include "Types/Types.hpp"
#include "Robot/RobotConfiguration.hpp"
#include "Robot/UniversalRobot/UniversalRobot.hpp"
#include "Robot/UniversalRobot/UniversalRobotConfiguration.hpp"

namespace crf::actuators::robot {

UniversalRobot::UniversalRobot(
    std::shared_ptr<IUniversalRobotRTDEInterface> urRtdeInterface,
    const UniversalRobotConfiguration& robotConfigFile):
    urRtdeInterface_(urRtdeInterface),
    robotConfiguration_(robotConfigFile),
    isInitialized_(false),
    logger_("UniversalRobot") {
    logger_->debug("CTor");
    profileParams_ = robotConfiguration_.getProfileParameters();
}

UniversalRobot::~UniversalRobot() {
    logger_->debug("DTor");
    deinitialize();
}

bool UniversalRobot::initialize() {
    logger_->debug("initialize");

    if (isInitialized_) {
        logger_->error("The API is already initialized");
        return false;
    }

    urRtdeInterface_->initRtdeControlInterface(robotConfiguration_.getIPAddress());
    logger_->info("RTDE control established with {}", robotConfiguration_.getIPAddress());
    urRtdeInterface_->initRtdeReceiveInterface(robotConfiguration_.getIPAddress());
    logger_->info("RTDE receive established with {}", robotConfiguration_.getIPAddress());

    if (!urRtdeInterface_->zeroFtSensor()) {
        logger_->error("Force sensor could not be zeroed");
        return false;
    }

    isInitialized_ = true;
    return true;
}

bool UniversalRobot::deinitialize() {
    logger_->debug("deinitialize");
    if (!isInitialized_) {
        logger_->error("The RTDE driver hasn't been initialized");
        return false;
    }
    softStop();
    urRtdeInterface_->stopScript();
    logger_->debug("The UR Script has been stopped");
    isInitialized_ = false;
    return true;
}

crf::expected<crf::utility::types::JointPositions> UniversalRobot::getJointPositions() {
    logger_->debug("getJointPositions");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    std::vector<double> qA = urRtdeInterface_->getActualQ();
    return utility::types::JointPositions({qA[0], qA[1], qA[2], qA[3], qA[4], qA[5]});
}

crf::expected<crf::utility::types::JointVelocities> UniversalRobot::getJointVelocities() {
    logger_->debug("getJointVelocities");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    std::vector<double> qdA = urRtdeInterface_->getActualQd();
    return utility::types::JointVelocities({qdA[0], qdA[1], qdA[2], qdA[3], qdA[4], qdA[5]});
}

crf::expected<crf::utility::types::JointAccelerations> UniversalRobot::getJointAccelerations() {
    logger_->debug("getJointAccelerations");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::JointForceTorques> UniversalRobot::getJointForceTorques() {
    logger_->debug("getJointForceTorques");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    std::vector<double> jT = urRtdeInterface_->getJointForceTorques();
    return crf::utility::types::JointForceTorques({jT[0], jT[1], jT[2], jT[3], jT[4], jT[5]});
}

crf::expected<crf::utility::types::TaskPose> UniversalRobot::getTaskPose() {
    logger_->debug("getTaskPose");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    std::vector<double> tP = urRtdeInterface_->getActualTCPPose();
    double angle = std::sqrt(std::pow(tP[3], 2) + std::pow(tP[4], 2) + std::pow(tP[5], 2));
    return utility::types::TaskPose(
        {tP[0], tP[1], tP[2]},
        Eigen::AngleAxisd(angle, Eigen::Vector3d({tP[3]/angle, tP[4]/angle, tP[5]/angle})));
}

crf::expected<crf::utility::types::TaskVelocity> UniversalRobot::getTaskVelocity() {
    logger_->debug("getTaskVelocity");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    std::vector<double> tS = urRtdeInterface_->getActualTCPSpeed();
    return utility::types::TaskVelocity({tS[0], tS[1], tS[2], tS[3], tS[4], tS[5]});
}

crf::expected<crf::utility::types::TaskAcceleration> UniversalRobot::getTaskAcceleration() {
    logger_->debug("getTaskAcceleration");
    return crf::Code::MethodNotAllowed;
}

crf::expected<crf::utility::types::TaskForceTorque> UniversalRobot::getTaskForceTorque() {
    logger_->debug("getTaskForceTorque");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }

    std::vector<double> fT = urRtdeInterface_->getActualTCPForce();
    return crf::utility::types::TaskForceTorque ({fT[0], fT[1], fT[2], fT[3], fT[4], fT[5]});
}

crf::expected<bool> UniversalRobot::setJointPositions(const bool& isSmoothTrajectory,
    const crf::utility::types::JointPositions& jointPositions,
    const crf::utility::types::JointVelocities& jointVelocities,
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->debug("setJointPositions");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }

    uint64_t nDoF = robotConfiguration_.getJointSpaceDoF();
    if ( jointPositions.size() != nDoF ) {
        logger_->error("Input position size not valid");
        return crf::Code::BadRequest;
    }

    crf::utility::types::JointVelocities jointVel{jointVelocities};
    crf::utility::types::JointAccelerations jointAcc{jointAccelerations};
    if (jointVel.size() != nDoF) {
        logger_->info("jointVelocities size is not matching with nDoF({}). "
            "Using default profile joints velocity.", nDoF);
        jointVel = profileParams_.jointVelocities;
    }
    if (jointAcc.size() != nDoF) {
        logger_->info("jointAccelerations size is not matching nDoF({}). "
            "Using default profile joints acceleration.", nDoF);
        jointAcc = profileParams_.jointAccelerations;
    }

    // Safety Check: Joint angles, velocities and accelerations within Limits?
    double maxAcceleration{0}, maxVelocity{0}, maxJointPos{0}, minJointPos{0};
    for (unsigned int jointID = 0; jointID < nDoF; jointID++) {
        maxJointPos = robotConfiguration_.getJointLimits().maxPosition[jointID];
        minJointPos = robotConfiguration_.getJointLimits().minPosition[jointID];
        maxVelocity = robotConfiguration_.getJointLimits().maxVelocity[jointID];
        maxAcceleration = robotConfiguration_.getJointLimits().maxAcceleration[jointID];

        if (jointPositions[jointID] < minJointPos) {
            logger_->error("The requested position ({}) for the joint {} is less than the lower "
                "joint limit", jointPositions[jointID], jointID+1);
            return crf::Code::BadRequest;
        }
        if (jointPositions[jointID] > maxJointPos) {
            logger_->error("The requested position ({}) for the joint {} is greater than the upper "
                "joint limit", jointPositions[jointID], jointID+1);
            return crf::Code::BadRequest;
        }
        if (jointAcc[jointID] <= 0 || jointAcc[jointID] >= maxAcceleration) {
            logger_->info("The requested acceleration ({}) for index {} is not in the interval ]0,"
                " {}]. Using default profile joints acceleration_.", jointAcc[jointID], jointID+1,
                 maxAcceleration);
            jointAcc = profileParams_.jointAccelerations;
        }
        if (jointVel[jointID] <= 0 || jointVel[jointID] > maxVelocity) {
            logger_->info("The requested velocity ({}) for index {} is not in the interval ]0, {}]"
                ".Using default profile joints velocity.", jointVel[jointID], jointID+1,
                maxVelocity);
            jointVel = profileParams_.jointVelocities;
        }
    }

    std::vector<double> qDes(nDoF, 0);
    for (unsigned int i = 0; i < nDoF; i++) {
      qDes[i] = jointPositions[i];
    }

    if (isSmoothTrajectory) {
        return urRtdeInterface_->servoJ(qDes, 0, 0, std::chrono::duration<double>(
            robotConfiguration_.getRobotControllerLoopTime()).count(),
            robotConfiguration_.getLookAheadTime(), robotConfiguration_.getGain());
    }
    double leadingVelocity{0}, leadingAcceleration{0};
    for (unsigned int i = 0; i < nDoF; i++) {
        if (leadingVelocity < jointVel[i]) {
            leadingVelocity = jointVel[i];
        }
        if (leadingAcceleration < jointAcc[i]) {
            leadingAcceleration = jointAcc[i];
        }
    }
    return urRtdeInterface_->moveJ(qDes, leadingVelocity, leadingAcceleration, false);
}

crf::expected<bool> UniversalRobot::setJointVelocities(const bool& isSmoothTrajectory,
    const crf::utility::types::JointVelocities& jointVelocities,
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->debug("setJointVelocities");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    uint64_t nDoF{robotConfiguration_.getJointSpaceDoF()};
    if (jointVelocities.size() != nDoF) {
        logger_->error("Input velocity size not valid");
        return crf::Code::BadRequest;
    }
    crf::utility::types::JointAccelerations jointsAcc;
    if (jointAccelerations.size() == 0) {
        logger_->debug("No Acceleration provided. Using default profile joints acceleration");
        jointsAcc = profileParams_.jointAccelerations;
    } else if (jointAccelerations.size() == nDoF) {
        jointsAcc = jointAccelerations;
    } else {
        logger_->error("Input Acceleration size not valid");
        return crf::Code::BadRequest;
    }
    // Check if the values of velocity are within the robot limits
    for (unsigned int jointID = 0; jointID < nDoF; jointID++) {
        if (std::fabs(jointVelocities[jointID]) >
            robotConfiguration_.getJointLimits().maxVelocity[jointID]) {
            logger_->error("The requested velocity ({}) for joint {} is bigger than the limit",
                jointVelocities[jointID], jointID+1);
            return crf::Code::BadRequest;
        }
    }

    if (!isSmoothTrajectory) {
        logger_->error("Non Smooth Velocity Control Not Implemented");
        return crf::Code::MethodNotAllowed;
    }

    std::vector<double> qddDes(nDoF, 0);
    for (unsigned int i = 0; i < nDoF; i++) {
        qddDes[i] = jointsAcc[i];
    }
    double maxAcceleration{*std::max_element(qddDes.begin(), qddDes.end())};
    std::vector<double> qdDes(nDoF, 0);
    for (unsigned int i = 0; i < nDoF; i++) {
        qdDes[i] = jointVelocities[i];
    }
    logger_->debug("Converted JointVelocities into Vector");
    return urRtdeInterface_->speedJ(qdDes, maxAcceleration, std::chrono::duration<double>(
        robotConfiguration_.getRobotControllerLoopTime()).count());
}

crf::expected<bool> UniversalRobot::setJointForceTorques(const bool& isSmoothTrajectory,
    const crf::utility::types::JointForceTorques& jointForceTorques) {
    logger_->debug("setJointForceTorques");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> UniversalRobot::setTaskPose(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskPose& taskPose,
    const crf::utility::types::TaskVelocity& taskVelocity,
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->debug("setTaskPose");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    unsigned int nDoF{6};
    Eigen::Vector3d zDesPosition = taskPose.getPosition();
    Eigen::AngleAxisd zDesAngleAxis = taskPose.getAngleAxis();
    std::vector<double> zDes{zDesPosition[0], zDesPosition[1], zDesPosition[2],
        zDesAngleAxis.angle()*zDesAngleAxis.axis()[0],
        zDesAngleAxis.angle()*zDesAngleAxis.axis()[1],
        zDesAngleAxis.angle()*zDesAngleAxis.axis()[2]};

    double maxVelocity{0}, maxAcceleration{0};
    bool useProfileTaskVelocity{false}, useProfileTaskAcceleration{false};
    for (unsigned int jointID = 0; jointID < nDoF; jointID++) {
        maxVelocity = robotConfiguration_.getTaskLimits().maxVelocity[jointID];
        maxAcceleration = robotConfiguration_.getTaskLimits().maxAcceleration[jointID];
        if (taskAcceleration[jointID] <= 0 || taskAcceleration[jointID] > maxAcceleration) {
            logger_->info("The requested acceleration ({}) for index {} is not in the interval ]0,"
                " {}] or was not provided explicitely. Using default profile task acceleration.",
                taskAcceleration[jointID], jointID+1, maxAcceleration);
            useProfileTaskAcceleration = true;
        }
        if (taskVelocity[jointID] <= 0 || taskVelocity[jointID] > maxVelocity) {
            logger_->info(
                "The requested velocity ({}) for index {} is not in the interval ]0, {}] or was "
                "not provided explicitely. Using default profile task velocity.",
                taskVelocity[jointID], jointID+1, maxVelocity);
            useProfileTaskVelocity = true;
        }
    }

    double tcpSpeed{0};
    if (useProfileTaskVelocity) {
        tcpSpeed = std::sqrt(
            std::pow(profileParams_.taskVelocity[0], 2) +
            std::pow(profileParams_.taskVelocity[1], 2) +
            std::pow(profileParams_.taskVelocity[2], 2));
    } else {
        tcpSpeed = std::sqrt(std::pow(taskVelocity[0], 2) +
            std::pow(taskVelocity[1], 2) + std::pow(taskVelocity[2], 2));
    }

    double tcpAcceleration{0};
    if (useProfileTaskAcceleration) {
        tcpAcceleration =  std::sqrt(
            std::pow(profileParams_.taskAcceleration[0], 2) +
            std::pow(profileParams_.taskAcceleration[1], 2) +
            std::pow(profileParams_.taskAcceleration[2], 2));
    } else {
        tcpAcceleration = std::sqrt(std::pow(taskAcceleration[0], 2) +
            std::pow(taskAcceleration[1], 2) + std::pow(taskAcceleration[2], 2));
    }

    if (isSmoothTrajectory) {
        logger_->error("Control of Smooth Task Trajectories is not available for UR10e");
        return crf::Code::MethodNotAllowed;
    }
    return urRtdeInterface_->moveL(zDes, tcpSpeed, tcpAcceleration, false);  // Asynchronous
}

crf::expected<bool> UniversalRobot::setTaskVelocity(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskVelocity& taskVelocity,
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->debug("setTaskVelocity");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    uint64_t nDoF{6};

    // Check if the values of velocity are within the robot limits
    for (unsigned int jointID = 0; jointID < nDoF; jointID++) {
        if (std::fabs(taskVelocity[jointID]) >
            robotConfiguration_.getTaskLimits().maxVelocity[jointID]) {
            logger_->error("The requested velocity ({}) for joint {} is bigger than the limit",
                taskVelocity[jointID], jointID+1);
            return crf::Code::BadRequest;
        }
    }

    if (!isSmoothTrajectory) {
        logger_->error("Non Smooth Velocity Control Not Available for UR10e");
        return crf::Code::MethodNotAllowed;
    }

    std::vector<double> zddDes(nDoF, 0);
    for (unsigned int i = 0; i < nDoF; i++) {
        zddDes[i] = taskAcceleration[i];
    }
    double leadingAcceleration{*std::max_element(zddDes.begin(), zddDes.end())};

    std::vector<double> zdDes(nDoF, 0);
    for (unsigned int i = 0; i < nDoF; i++) {
        zdDes[i] = taskVelocity[i];
    }

    return urRtdeInterface_->speedL(zdDes, leadingAcceleration, std::chrono::duration<double>(
        robotConfiguration_.getRobotControllerLoopTime()).count());
}

crf::expected<bool> UniversalRobot::setTaskForceTorque(const bool& isSmoothTrajectory,
    const crf::utility::types::TaskForceTorque& taskForceTorque) {
    logger_->debug("setTaskForceTorque");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    uint64_t nDoF{6};

    std::vector<double> forceFrame = urRtdeInterface_->getActualTCPPose();
    std::vector<int> complianceSelector {0, 0, 0, 0, 0, 0};
    int transformTyp{2};
    std::vector<double> limits{0.1, 0.1, 0.1, 0.785, 0.785, 0.785};
    std::vector<double> desiredForceTorque(nDoF, 0);
    for (unsigned int i = 0; i < nDoF; i++) {
        desiredForceTorque[i] = taskForceTorque[i];
    }

    if (!isSmoothTrajectory) {
        logger_->error("Non Smooth Task Torque Control Not Implemented");
        return crf::Code::MethodNotAllowed;
    }
    return urRtdeInterface_->forceMode(forceFrame, complianceSelector, desiredForceTorque,
        transformTyp, limits);
}

crf::expected<crf::utility::types::JointVelocities> UniversalRobot::getProfileJointVelocities() {  // NOLINT
    logger_->debug("getProfileJointVelocities");
    return profileParams_.jointVelocities;
}

crf::expected<crf::utility::types::JointAccelerations> UniversalRobot::getProfileJointAccelerations() {  // NOLINT
    logger_->debug("getProfileJointAccelerations");
    return profileParams_.jointAccelerations;
}

crf::expected<crf::utility::types::TaskVelocity> UniversalRobot::getProfileTaskVelocity() {
    logger_->debug("getProfileTaskVelocity");
    return profileParams_.taskVelocity;
}

crf::expected<crf::utility::types::TaskAcceleration> UniversalRobot::getProfileTaskAcceleration() {  // NOLINT
    logger_->debug("getProfileTaskAcceleration");
    return profileParams_.taskAcceleration;
}

crf::expected<bool> UniversalRobot::setProfileJointVelocities(
    const crf::utility::types::JointVelocities& jointVelocities) {
    logger_->debug("setProfileJointVelocities");
    uint64_t nDoF{robotConfiguration_.getJointSpaceDoF()};
    if (jointVelocities.size() != nDoF) {
        logger_->error("Input velocity size not valid");
        return crf::Code::BadRequest;
    }
    double maxVelocity{0};
    for (int i = 0; i < jointVelocities.size(); i++) {
        maxVelocity = robotConfiguration_.getJointLimits().maxVelocity[i];
        if (jointVelocities[i] <= 0 || jointVelocities[i] > maxVelocity) {
            logger_->error("The requested velocity ({}) for index {} is not in the interval ]0, {}"
                "] or was not provided explicitely. Profile joint velocity remains unchanged.",
                jointVelocities[i], i+1, maxVelocity);
            return crf::Code::BadRequest;
        }
    }
    profileParams_.jointVelocities = jointVelocities;
    return true;
}

crf::expected<bool> UniversalRobot::setProfileJointAccelerations(
    const crf::utility::types::JointAccelerations& jointAccelerations) {
    logger_->debug("setProfileJointAccelerations");
    uint64_t nDoF{robotConfiguration_.getJointSpaceDoF()};
    if (jointAccelerations.size() != nDoF) {
        logger_->error("Input acceleration size not valid");
        return crf::Code::BadRequest;
    }
    double maxAcceleration{0};
    for (int i = 0; i < jointAccelerations.size(); i++) {
        maxAcceleration = robotConfiguration_.getJointLimits().maxAcceleration[i];
        if (jointAccelerations[i] <= 0 || jointAccelerations[i] > maxAcceleration) {
            logger_->error("The requested acceleration ({}) for index {} is not in the interval ]0"
                ", {}] or was not provided explicitely. Profile joint acceleration remains "
                "unchanged.", jointAccelerations[i], i+1, maxAcceleration);
            return crf::Code::BadRequest;
        }
    }
    profileParams_.jointAccelerations = jointAccelerations;
    return true;
}

crf::expected<bool> UniversalRobot::setProfileTaskVelocity(
    const crf::utility::types::TaskVelocity& taskVelocity) {
    logger_->debug("setProfileTaskVelocity");
    double maxVelocity{0};
    for (int i = 0; i < taskVelocity.size(); i++) {
        maxVelocity = robotConfiguration_.getTaskLimits().maxVelocity[i];
        if (taskVelocity[i] <= 0 || taskVelocity[i] > maxVelocity) {
            logger_->error("The requested velocity ({}) for index {} is not in the interval ]0, {}"
                "] or was not provided explicitely. Profile task velocity remains unchanged.",
                taskVelocity[i], i+1, maxVelocity);
            return crf::Code::BadRequest;
        }
    }
    profileParams_.taskVelocity = taskVelocity;
    return true;
}

crf::expected<bool> UniversalRobot::setProfileTaskAcceleration(
    const crf::utility::types::TaskAcceleration& taskAcceleration) {
    logger_->debug("setProfileTaskAcceleration");
    double maxAcceleration{0};
    for (int i = 0; i < taskAcceleration.size(); i++) {
        maxAcceleration = robotConfiguration_.getTaskLimits().maxAcceleration[i];
        if (taskAcceleration[i] <= 0 || taskAcceleration[i] > maxAcceleration) {
            logger_->error("The requested acceleration ({}) for index {} is not in the interval ]0"
                ", {}] or was not provided explicitely. Profile joint acceleration remains "
                "unchanged.", taskAcceleration[i], i+1, maxAcceleration);
            return crf::Code::BadRequest;
        }
    }
    profileParams_.taskAcceleration = taskAcceleration;
    return true;
}

crf::expected<bool> UniversalRobot::setGravity(const std::array<double, 3>& gravity) {
    double gravityMagnitude = sqrt(pow(gravity[0], 2) + pow(gravity[1], 2) + pow(gravity[2], 2));
    if (abs(gravityMagnitude - 9.81) > 0.1) {
        logger_->error("Where are you? The magnitude of the gravity vector is not"
            " in [9.71, 9.91] m/s^2");
        return crf::Code::BadRequest;
    }
    std::vector<double> gravity4UrRtde(gravity.begin(), gravity.end());

    return urRtdeInterface_->setGravity(gravity4UrRtde);
}

crf::expected<bool> UniversalRobot::softStop() {
    logger_->debug("softStop");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    return urRtdeInterface_->speedStop();
}

crf::expected<bool> UniversalRobot::hardStop() {
    logger_->debug("hardStop");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> UniversalRobot::setBrakes(std::vector<bool> brakesStatus) {
    logger_->debug("setBrakes");
    return crf::Code::MethodNotAllowed;
}

crf::expected<std::vector<bool>> UniversalRobot::getBrakes() {
    logger_->debug("getBrakes");
    return crf::Code::MethodNotAllowed;
}

std::set<Code> UniversalRobot::robotStatus() {
    logger_->debug("robotStatus");
    std::set<Code> statusList;

    if (!isInitialized_) {
        statusList.insert(crf::Code::NotInitialized);
    }

    uint32_t status = urRtdeInterface_->getRobotStatus();
    if (status & static_cast<uint32_t>(1)) {
        statusList.insert(crf::Code::PoweredOn);
    }
    if (status & static_cast<uint32_t>(2)) {
        statusList.insert(crf::Code::UR_ProgramRunning);
    }
    if (status & static_cast<uint32_t>(4)) {
        statusList.insert(crf::Code::UR_TeachButtonPressed);
    }
    if (status & static_cast<uint32_t>(8)) {
        statusList.insert(crf::Code::UR_PowerButtonPressed);
    }

    int32_t mode = urRtdeInterface_->getRobotMode();
    if (mode == -1) {
        statusList.insert(crf::Code::UR_NoController);
    } else if (mode == 0) {
        statusList.insert(crf::Code::Disconnected);
    } else if (mode == 1) {
        statusList.insert(crf::Code::UR_ConfirmSafety);
    } else if (mode == 2) {
        statusList.insert(crf::Code::UR_Booting);
    } else if (mode == 3) {
        statusList.insert(crf::Code::PoweredOff);
    } else if (mode == 4) {
        statusList.insert(crf::Code::PoweredOn);
    } else if (mode == 5) {
        statusList.insert(crf::Code::Idle);
    } else if (mode == 6) {
        statusList.insert(crf::Code::UR_BackDrive);
    } else if (mode == 7) {
        statusList.insert(crf::Code::UR_Running);
    } else if (mode == 8) {
        statusList.insert(crf::Code::UR_UpdatingFirmware);
    } else {
        logger_->error("Robot Mode not Recognized");
    }

    uint32_t safetyStatus = urRtdeInterface_->getSafetyStatusBits();
    if (status & static_cast<uint32_t>(1)) {
        statusList.insert(crf::Code::UR_NormalMode);
    }
    if (status & static_cast<uint32_t>(2)) {
        statusList.insert(crf::Code::UR_ReducedMode);
    }
    if (status & static_cast<uint32_t>(4)) {
        statusList.insert(crf::Code::UR_ProtectiveStop);
    }
    if (status & static_cast<uint32_t>(8)) {
        statusList.insert(crf::Code::UR_RecoveryMode);
    }
    if (status & static_cast<uint32_t>(16)) {
        statusList.insert(crf::Code::UR_SafeguardStop);
    }
    if (status & static_cast<uint32_t>(32)) {
        statusList.insert(crf::Code::UR_SystemEmergencyStop);
    }
    if (status & static_cast<uint32_t>(64)) {
        statusList.insert(crf::Code::UR_RobotEmergencyStop);
    }
    if (status & static_cast<uint32_t>(128)) {
        statusList.insert(crf::Code::EmergencyStop);
    }
    if (status & static_cast<uint32_t>(256)) {
        statusList.insert(crf::Code::UR_Violation);
    }
    if (status & static_cast<uint32_t>(512)) {
        statusList.insert(crf::Code::FaultState);
    }
    if (status & static_cast<uint32_t>(1024)) {
        statusList.insert(crf::Code::UR_StoppedDueToSafety);
    }

    return statusList;
}

crf::expected<bool> UniversalRobot::resetFaultState() {
    logger_->debug("resetFaultState");
    return crf::Code::MethodNotAllowed;
}

std::shared_ptr<RobotConfiguration> UniversalRobot::getConfiguration() {
    logger_->debug("getConfiguration");
    return std::make_shared<RobotConfiguration>(robotConfiguration_);
}

}  // namespace crf::actuators::robot
