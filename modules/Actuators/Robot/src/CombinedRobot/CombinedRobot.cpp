/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Hannes Gamper CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include "crf/expected.hpp"
#include "Types/Types.hpp"
#include "Robot/IRobot.hpp"
#include "Robot/CombinedRobot/CombinedRobot.hpp"
#include "Robot/CombinedRobot/CombinedRobotConfiguration.hpp"

using crf::utility::types::TaskPose;
using crf::utility::types::TaskVelocity;
using crf::utility::types::TaskAcceleration;
using crf::utility::types::TaskForceTorque;
using crf::utility::types::JointPositions;
using crf::utility::types::JointVelocities;
using crf::utility::types::JointAccelerations;
using crf::utility::types::JointForceTorques;

namespace crf::actuators::robot {

CombinedRobot::CombinedRobot(std::vector<std::shared_ptr<IRobot>> robots,
    const CombinedRobotConfiguration& configuration):
    robots_(robots),
    robotConfiguration_(configuration),
    isInitialized_(false),
    logger_("CombinedRobot") {
    logger_->debug("CTor");
    if (robots.size() < 2) {
        throw std::runtime_error("CombinedRobot - Not allowed with < 2 robots.");
    }
    robotsDof_ = robotConfiguration_.getJointDimensionsOfRobots();
    nRobots_ = robotConfiguration_.getNumberOfRobots();
    combinedDof_ = robotConfiguration_.getJointSpaceDoF();
    profileParams_ = robotConfiguration_.getProfileParameters();
}

CombinedRobot::~CombinedRobot() {
    logger_->debug("DTor");
    if (isInitialized_) deinitialize();
}

bool CombinedRobot::initialize() {
    logger_->debug("initialize");
    if (isInitialized_) {
        logger_->error("The CombinedRobot has already been initialized");
        return false;
    }
    for (unsigned int i = 0; i < nRobots_; i++) {
        if (robots_[i]->initialize()) continue;
        logger_->error("Robot {} couldn't be initialized!", i);
        deinitialize();
        return false;
    }
    isInitialized_ = true;
    return true;
}

bool CombinedRobot::deinitialize() {
    logger_->debug("deinitialize");
    if (!isInitialized_) {
        logger_->error("The CombinedRobot hasn't been initialized");
        return false;
    }
    bool ret {true};
    for (unsigned int i = 0; i < nRobots_; i++) {
        if (!robots_[i]->deinitialize()) {
            logger_->error("Robot {} couldn't be deinitialized!", i);
            ret = false;
        }
    }
    isInitialized_ = false;
    return ret;
}

crf::expected<JointPositions> CombinedRobot::getJointPositions() {
    logger_->debug("getJointPositions");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }

    JointPositions q(combinedDof_);
    crf::expected<JointPositions> responseExpected;
    unsigned int count {0};
    for (unsigned int i = 0; i < nRobots_; i++) {
        responseExpected = robots_[i]->getJointPositions();
        JointPositions response;
        if (!responseExpected) {
            logger_->error("Robot {} failed to return joint angles!", i);
            return responseExpected;
        }
        response = responseExpected.value();
        for (int j = 0; j < response.size(); j++) {
            q[count + j] = response[j];
        }
        count += robotsDof_[i];
    }
    return q;
}

crf::expected<JointVelocities> CombinedRobot::getJointVelocities() {
    logger_->debug("getJointVelocities");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }

    JointVelocities qd(combinedDof_);
    crf::expected<JointVelocities> responseExpected;
    unsigned int count {0};
    for (unsigned int i=0; i < nRobots_; i++) {
        responseExpected = robots_[i]->getJointVelocities();
        JointVelocities response;
        if (!responseExpected) {
            logger_->error("Robot {} failed to return joint velocities!", i);
            return responseExpected;
        }
        response = responseExpected.value();
        for (int j = 0; j < response.size(); j++) {
            qd[count + j] = response[j];
        }
        count += robotsDof_[i];
    }
    return qd;
}

crf::expected<JointAccelerations> CombinedRobot::getJointAccelerations() {
    logger_->debug("getJointAccelerations");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }

    JointAccelerations qdd(combinedDof_);
    crf::expected<JointAccelerations> responseExpected;
    unsigned int count {0};
    for (unsigned int i=0; i < nRobots_; i++) {
        responseExpected = robots_[i]->getJointAccelerations();
        JointAccelerations response;
        if (!responseExpected) {
            logger_->error("Robot {} failed to return joint accelerations!", i);
            return responseExpected;
        }
        response = responseExpected.value();
        for (int j = 0; j < response.size(); j++) {
            qdd[count + j] = response[j];
        }
        count += robotsDof_[i];
    }
    return qdd;
}

crf::expected<JointForceTorques> CombinedRobot::getJointForceTorques() {
    logger_->debug("getJointForceTorques");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }

    JointForceTorques forceTorque(combinedDof_);
    crf::expected<JointForceTorques> responseExpected;
    unsigned int count {0};
    for (unsigned int i=0; i < nRobots_; i++) {
        responseExpected = robots_[i]->getJointForceTorques();
        JointForceTorques response;
        if (!responseExpected) {
            logger_->error("Robot {} failed to return joint torques!", i);
            return responseExpected;
        }
        response = responseExpected.value();
        for (int j = 0; j < response.size(); j++) {
            forceTorque[count + j] = response[j];
        }
        count += robotsDof_[i];
    }
    return forceTorque;
}

crf::expected<TaskPose> CombinedRobot::getTaskPose() {
    logger_->debug("getTaskPose");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    return crf::Code::MethodNotAllowed;
}

crf::expected<TaskVelocity> CombinedRobot::getTaskVelocity() {
    logger_->debug("getTaskVelocity");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    return crf::Code::MethodNotAllowed;
}

crf::expected<TaskAcceleration> CombinedRobot::getTaskAcceleration() {
    logger_->debug("getTaskAcceleration");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    return crf::Code::MethodNotAllowed;
}

crf::expected<TaskForceTorque> CombinedRobot::getTaskForceTorque() {
    logger_->debug("getTaskForceTorque");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> CombinedRobot::setJointPositions(const bool& isSmoothTrajectory,
    const JointPositions& jointPositions, const JointVelocities& jointVelocities,
    const JointAccelerations& jointAccelerations) {
    logger_->debug("setJointPositions");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }

    if (jointPositions.size() != combinedDof_) {
        logger_->error("Input position size not valid");
        return crf::Code::BadRequest;
    }
    if (jointVelocities.size() != combinedDof_ && jointVelocities.size() != 0) {
        logger_->info("jointVelocities size is not matching with nDoF({}).", combinedDof_);
        return crf::Code::BadRequest;
    }
    if (jointAccelerations.size() != combinedDof_ && jointAccelerations.size() != 0) {
        logger_->info("jointAccelerations size is not matching nDoF({}). ", combinedDof_);
        return crf::Code::BadRequest;
    }

    // probably best to create the the vectors in the constructor
    unsigned int count = 0;
    crf::expected<bool> response;
    for (unsigned int i = 0; i < nRobots_; i++) {
        JointPositions q(robotsDof_[i]);
        crf::expected<bool> response;

        for (int j = 0; j < robotsDof_[i]; j++) {
            q[j] = jointPositions[count + j];
        }

        if (jointVelocities.size() == 0) {
            response = robots_[i]->setJointPositions(isSmoothTrajectory, q);
            count += robotsDof_[i];
            continue;
        }

        JointVelocities qd(robotsDof_[i]);

        for (int j = 0; j < robotsDof_[i]; j++) {
            qd[j] = jointVelocities[count + j];
        }

        if (jointAccelerations.size() == 0) {
            response = robots_[i]->setJointPositions(isSmoothTrajectory, q, qd);
            count += robotsDof_[i];
            continue;
        }

        JointAccelerations qdd(robotsDof_[i]);

        for (int j = 0; j < robotsDof_[i]; j++) {
            qdd[j] = jointAccelerations[count + j];
        }

        response = robots_[i]->setJointPositions(isSmoothTrajectory, q, qd, qdd);
        count += robotsDof_[i];

        if (!response) {
            logger_->error("Robot {} failed to set joint positions!", i);
            softStop();
            return response;
        }
    }
    return true;
}

crf::expected<bool> CombinedRobot::setJointVelocities(const bool& isSmoothTrajectory,
    const JointVelocities& jointVelocities, const JointAccelerations& jointAccelerations) {
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }

    if (jointVelocities.size() != combinedDof_) {
        logger_->info("jointVelocities size is not matching with nDoF({}). "
            "Using default profile joints velocity.", combinedDof_);
        return crf::Code::BadRequest;
    }
    if (jointAccelerations.size() != combinedDof_ && jointAccelerations.size() != 0) {
        logger_->info("jointAccelerations size is not matching nDoF({}). "
            "Using default profile joints acceleration.", combinedDof_);
        return crf::Code::BadRequest;
    }

    unsigned int count = 0;
    crf::expected<bool> response;
    for (unsigned int i=0; i < nRobots_; i++) {
        JointVelocities qd(robotsDof_[i]);

        for (int j = 0; j < robotsDof_[i]; j++) {
            qd[j] = jointVelocities[count + j];
        }

        if (jointAccelerations.size() == 0) {
            response = robots_[i]->setJointVelocities(isSmoothTrajectory, qd);
            count += robotsDof_[i];
            continue;
        }

        JointAccelerations qdd(robotsDof_[i]);

        for (int j = 0; j < robotsDof_[i]; j++) {
            qdd[j] = jointAccelerations[count + j];
        }

        response = robots_[i]->setJointVelocities(isSmoothTrajectory, qd, qdd);
        count += robotsDof_[i];

        if (!response) {
            logger_->error("Robot {} failed to set joint positions!", i);
            softStop();
            return response;
        }
    }
    return true;
}

crf::expected<bool> CombinedRobot::setJointForceTorques(const bool& isSmoothTrajectory,
    const JointForceTorques& jointForceTorques) {
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }

    if (jointForceTorques.size() != combinedDof_) {
        logger_->info("jointAccelerations size is not matching nDoF({}). "
            "Using default profile joints acceleration.", combinedDof_);
        return crf::Code::BadRequest;
    }

    // probably best to create the the vectors in the constructor
    unsigned int count = 0;
    crf::expected<bool> response;
    for (unsigned int i=0; i < nRobots_; i++) {
        JointForceTorques t(robotsDof_[i]);

        for (int j = 0; j < robotsDof_[i]; j++) {
            t[j] = jointForceTorques[count + j];
        }

        response = robots_[i]->setJointForceTorques(isSmoothTrajectory, t);
        count += robotsDof_[i];
        if (!response) {
            logger_->error("Robot {} failed to set joint positions!", i);
            softStop();
            return response;
        }
    }
    return true;
}

crf::expected<bool> CombinedRobot::setTaskPose(const bool& isSmoothTrajectory,
    const TaskPose& taskPose, const TaskVelocity& taskVelocity,
    const TaskAcceleration& taskAcceleration) {
    logger_->debug("setTaskPose");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> CombinedRobot::setTaskVelocity(const bool& isSmoothTrajectory,
    const TaskVelocity& taskVelocity, const TaskAcceleration& taskAcceleration) {
    logger_->debug("setTaskVelocity");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> CombinedRobot::setTaskForceTorque(const bool& isSmoothTrajectory,
    const TaskForceTorque& taskForceTorque) {
    logger_->debug("setTaskForceTorque");
    return crf::Code::MethodNotAllowed;
}

crf::expected<JointVelocities> CombinedRobot::getProfileJointVelocities() {
    logger_->debug("getProfileJointVelocities");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }

    JointVelocities qd(combinedDof_);
    crf::expected<JointVelocities> responseExpected;
    unsigned int count {0};
    for (unsigned int i=0; i < nRobots_; i++) {
        responseExpected = robots_[i]->getProfileJointVelocities();
        JointVelocities response;
        if (!responseExpected) {
            logger_->error("Robot {} failed to return joint velocities!", i);
            return responseExpected;
        }
        response = responseExpected.value();
        for (int j = 0; j < response.size(); j++) {
            qd[count + j] = response[j];
        }
        count += robotsDof_[i];
    }
    return qd;
}

crf::expected<JointAccelerations> CombinedRobot::getProfileJointAccelerations() {
    logger_->debug("getProfileJointAccelerations");
    if (!isInitialized_) {
        return crf::Code::NotInitialized;
    }

    JointAccelerations qdd(combinedDof_);
    crf::expected<JointAccelerations> responseExpected;
    unsigned int count {0};
    for (unsigned int i=0; i < nRobots_; i++) {
        responseExpected = robots_[i]->getProfileJointAccelerations();
        JointAccelerations response;
        if (!responseExpected) {
            logger_->error("Robot {} failed to return profile joint accelerations!", i);
            return responseExpected;
        }
        response = responseExpected.value();
        for (int j = 0; j < response.size(); j++) {
            qdd[count + j] = response[j];
        }
        count += robotsDof_[i];
    }
    return qdd;
}

crf::expected<TaskVelocity> CombinedRobot::getProfileTaskVelocity() {
    logger_->debug("getProfileTaskVelocity");
    return crf::Code::MethodNotAllowed;
}

crf::expected<TaskAcceleration> CombinedRobot::getProfileTaskAcceleration() {
    logger_->debug("getProfileTaskAcceleration");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> CombinedRobot::setProfileJointVelocities(
    const JointVelocities& jointVelocities) {
    logger_->debug("setProfileJointVelocities");

    unsigned int count = 0;
    crf::expected<bool> response;
    for (unsigned int i=0; i < nRobots_; i++) {
        JointVelocities qd(robotsDof_[i]);

        for (int j = 0; j < robotsDof_[i]; j++) {
            qd[j] = jointVelocities[count + j];
        }

        response = robots_[i]->setProfileJointVelocities(qd);

        count += robotsDof_[i];

        if (!response) {
            logger_->error("Robot {} failed to set profile joint velocity!", i);
            softStop();
            return response;
        }
    }
    return true;
}

crf::expected<bool> CombinedRobot::setProfileJointAccelerations(
    const JointAccelerations& jointAccelerations) {
    logger_->debug("setProfileJointAccelerations");

    unsigned int count = 0;
    crf::expected<bool> response;
    for (unsigned int i=0; i < nRobots_; i++) {
        JointAccelerations qdd(robotsDof_[i]);

        for (int j = 0; j < robotsDof_[i]; j++) {
            qdd[j] = jointAccelerations[count + j];
        }

        response = robots_[i]->setProfileJointAccelerations(qdd);
        count += robotsDof_[i];
        if (!response) {
            logger_->error("Robot {} failed to set profile joint acceleration!", i);
            softStop();
            return response;
        }
    }
    return true;
}

crf::expected<bool> CombinedRobot::setProfileTaskVelocity(
    const TaskVelocity& taskVelocity) {
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> CombinedRobot::setProfileTaskAcceleration(
    const TaskAcceleration& taskAcceleration) {
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> CombinedRobot::setGravity(const std::array<double, 3>& gravity) {
    logger_->debug("setGravity");

    if (!isInitialized_) {
        logger_->error("The CombinedRobot hasn't been initialized");
        return crf::Code::NotInitialized;
    }

    std::vector<double> g;
    crf::expected<bool> response;
    crf::expected<TaskPose> zExpected;
    TaskPose gCombinedRobot(
        {gravity[0], gravity[1], gravity[2]},
        crf::math::rotation::CardanXYZ({0, 0, 0}));

    double gravityMagnitude = sqrt(
        std::pow(gravity[0], 2) + std::pow(gravity[1], 2) + std::pow(gravity[2], 2));

    if (std::abs(gravityMagnitude - 9.81) > 0.1) {
        logger_->error("Gravity vector is not from earth: ||g-9.81||>0.1!", gravity.size());
        return crf::Code::BadRequest;
    }

    response = robots_[0]->setGravity(gravity);
    if (!response) {
        logger_->error("Robot {} couldn't set gravity vector!", 0);
        return response;
    }
    for (unsigned int i=0; i < nRobots_-1; i++) {
        zExpected = robots_[i]->getTaskPose();

        if (!zExpected) {
            logger_->error("Robot {} couldn't return task position!", i);
            return zExpected;
        }
        TaskPose z = zExpected.value();

        if (z.getOrientationRepresentation() !=
            crf::math::rotation::OrientationRepresentation::Quaternion) {
            z = TaskPose(z.getPosition(), z.getQuaternion());
        }
        // A TaskPose object is being used to perform the transformation to the endeffector
        // of the first robot. The transformation is performed with homogenious transformation
        // matrices. This means that the position values have to be set to 0 in order to only
        // perform a change in orientation:
        z.setPosition(Eigen::Vector3d({0.0, 0.0, 0.0}));
        TaskPose gTemp(multiply(invert(z), gCombinedRobot));
        response = robots_[i+1]->setGravity(
            crf::utility::types::stdArrayFromEigenVector<3>(gTemp.getPosition()));
        if (!response) {
            logger_->error("Robot {} couldn't set gravity vector!", i);
            return response;
        }
    }
    return true;
}

crf::expected<bool> CombinedRobot::softStop() {
    logger_->debug("softStop");
    if (!isInitialized_) {
        logger_->error("The CombinedRobot hasn't been initialized");
        return crf::Code::NotInitialized;
    }

    crf::expected<bool> response, ret = true;
    for (unsigned int i=0; i < nRobots_; i++) {
        response = robots_[i]->softStop();
        if (!response) {
            logger_->error("Robot {} couldn't be soft stopped!", i);
            ret = response;
        }
    }
    return ret;
}

crf::expected<bool> CombinedRobot::hardStop() {
    logger_->debug("hardStop");
    if (!isInitialized_) {
        logger_->error("The CombinedRobot hasn't been initialized");
        return crf::Code::NotInitialized;
    }

    crf::expected<bool> response, ret = true;
    for (unsigned int i=0; i < nRobots_; i++) {
        response = robots_[i]->hardStop();
        if (!response) {
            logger_->error("Robot {} couldn't be hard stopped!", i);
            ret = response;
        }
    }
    return ret;
}

crf::expected<bool> CombinedRobot::setBrakes(std::vector<bool> brakesStatus) {
    logger_->debug("setBrakes");
    if (!isInitialized_) {
        logger_->error("The CombinedRobot hasn't been initialized");
        return crf::Code::NotInitialized;
    }

    // std::vector<bool> separateBrakesStatus;
    unsigned int count = 0;
    crf::expected<bool> response, ret = true;
    for (unsigned int i=0; i < nRobots_; i++) {
        std::vector<bool> separateBrakesStatus(robotsDof_[i]);
        std::copy(brakesStatus.begin() + count,
            brakesStatus.begin() + count + robotsDof_[i], separateBrakesStatus.begin());
        count += robotsDof_[i];

        response = robots_[i]->setBrakes(separateBrakesStatus);
        if (!response) {
            logger_->error("Robot {} failed to set brakes!", i);
            softStop();
            ret = response;
        }
    }
    return ret;
}

crf::expected<std::vector<bool>> CombinedRobot::getBrakes() {
    logger_->debug("getBrakes");
    if (!isInitialized_) {
        logger_->error("The CombinedRobot hasn't been initialized");
        return crf::Code::NotInitialized;
    }

    std::vector<bool> brakeStatus(combinedDof_);
    crf::expected<std::vector<bool>> response;
    unsigned int count {0};
    for (unsigned int i=0; i < nRobots_; i++) {
        response = robots_[i]->getBrakes();
        if (!response) {
            logger_->error("Robot {} failed to return brakes status!", i);
            return response;
        }
        std::copy(response.value().begin(), response.value().end(), brakeStatus.begin()+count);
        count += robotsDof_[i];
    }
    return brakeStatus;
}

std::set<Code> CombinedRobot::robotStatus() {
    logger_->debug("robotStatus");
    std::set<Code> status {robots_[0]->robotStatus()};
    for (unsigned int i=1; i < nRobots_; i++) {
        status.merge(robots_[i]->robotStatus());
    }
    return status;
}

crf::expected<bool> CombinedRobot::resetFaultState() {
    logger_->debug("resetFaultState");

    if (!isInitialized_) {
        logger_->error("The CombinedRobot hasn't been initialized");
        return crf::Code::NotInitialized;
    }
    crf::expected<bool> response, ret = true;
    for (unsigned int i=0; i < nRobots_; i++) {
        response = robots_[i]->resetFaultState();
        if (!response) {
            logger_->error("Robot {} couldn't be reset!", i);
            ret = response;
        }
    }

    return ret;
}

std::shared_ptr<RobotConfiguration> CombinedRobot::getConfiguration() {
    logger_->debug("getConfiguration");
    return std::make_shared<RobotConfiguration>(robotConfiguration_);
}

}  // namespace crf::actuators::robot
