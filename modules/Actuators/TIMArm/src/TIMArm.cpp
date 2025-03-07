/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 * Contributor: Francesco Riccardi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <nlohmann/json.hpp>
#include <memory>
#include <vector>
#include <string>
#include <boost/optional.hpp>

#include "TIMArm/TIMArm.hpp"
#include "KinovaArm/IKinovaApiInterface.hpp"
#include "TIMArm/TIMArmConfiguration.hpp"
#include "EventLogger/EventLogger.hpp"
#include "RobotArm/RobotArmConfiguration.hpp"
#include "KinovaArm/KinovaJaco.hpp"
#include "Types/Types.hpp"

#define VALID_POS_DIFF 0.02

namespace crf::actuators::timarm {

TIMArm::TIMArm(const nlohmann::json& armConfigFile,
    std::shared_ptr<crf::devices::ethercatdevices::TIMRobotArmWagonMotors> timMotors,
    std::shared_ptr<crf::actuators::kinovaarm::IKinovaApiInterface> kinovaAPI) :
    armConfigFile_(armConfigFile),
    timMotors_(timMotors),
    kinovaAPI_(kinovaAPI),
    logger_("TIMArm"),
    armConfiguration_(new crf::actuators::timarm::TIMArmConfiguration),
    etherCATArm_(nullptr),
    kinovaArm_(nullptr),
    isInitialized_(false) {
    logger_->debug("CTor");
}

TIMArm::~TIMArm() {
    logger_->debug("DTor");
}

bool TIMArm::initialize() {
    logger_->debug("initialize");
    if (isInitialized_) {
        logger_->error("The robot is already initialized");
        return false;
    }
    if (!armConfiguration_->parse(armConfigFile_)) {
        logger_->error("Failed to read the configuration file");
        return false;
    }

    std::optional<std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor>> temp;
    temp = timMotors_->getHarmonicDrive1();
    if (!temp) {
        std::cout << "Cannot retrieve EtherCAT drive 1" << std::endl;
        return -1;
    }
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> HD1 = temp.value();
    temp = timMotors_->getHarmonicDrive2();
    if (!temp) {
        std::cout << "Cannot retrieve EtherCAT drive 2" << std::endl;
        return -1;
    }
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> HD2 = temp.value();
    temp = timMotors_->getLinearSled();
    if (!temp) {
        std::cout << "Cannot retrieve linear sled" << std::endl;
        return -1;
    }
    std::shared_ptr<crf::devices::ethercatdevices::IEtherCATMotor> linearSled = temp.value();

    try {
        etherCATArm_.reset(new crf::actuators::ethercatrobotarm::EtherCATRobotArm(
            armConfiguration_->getHarmonicArmJSON(), HD1, HD2, linearSled));
    }
    catch (const std::exception& e) {
        logger_->error("Failed to create the EtherCAT robot arm object");
        return false;
    }
    if (!etherCATArm_->initialize()) {
        logger_->error("Failed to initialize the EtherCAT robot arm");
        return false;
    }

    try {
        kinovaArm_.reset(new crf::actuators::kinovaarm::KinovaJaco(kinovaAPI_,
            armConfiguration_->getKinovaArmJSON()));
    } catch (const std::exception& e) {
        logger_->error("Failed to create the Kinova robot arm object");
        return false;
    }
    if (!kinovaArm_->initialize()) {
        logger_->error("Failed to initialize the Kinova arm");
        return false;
    }

    logger_->info("The TIM arm is initialized");
    isInitialized_ = true;
    return true;
}

bool TIMArm::deinitialize() {
    logger_->debug("deinitialize");
    if (!isInitialized_) {
        logger_->error("The robot hasn't been initialized");
        return false;
    }
    if (!stopArm()) {
        logger_->error("Failed to stop the robot arm");
        return false;
    }
    if (!etherCATArm_->deinitialize()) {
        logger_->error("Failed to initialize the EtherCAT robot arm");
        return false;
    }
    if (!kinovaArm_->deinitialize()) {
        logger_->error("Failed to deinitialize the Kinova arm");
        return false;
    }
    logger_->info("The TIM arm is deinitilized");
    isInitialized_ = false;
    return true;
}

boost::optional<crf::utility::types::JointPositions> TIMArm::getJointPositions() {
    logger_->debug("getJointPositions");
    if (!isInitialized_) {
        logger_->error("The robot hasn't been initialized - Returning false");
        return boost::none;
    }
    //  We consider that the offset and direction defined for each joint in the configuration file
    //  is applied internally on each robot class, that is why we return directly the received.
    //  position.
    auto jointPositionsHarmonicArm = etherCATArm_->getJointPositions();
    if (!jointPositionsHarmonicArm) {
        logger_->error("Failed to get the position from the EtherCAT robot arm - Returning false");
        return boost::none;
    }
    auto jointPositionsKinovaArm = kinovaArm_->getJointPositions();
    if (!jointPositionsKinovaArm) {
        logger_->error("Failed to get the position from the Kinova arm - Returning false");
        return boost::none;
    }
    return combinePositions(jointPositionsHarmonicArm.get(), jointPositionsKinovaArm.get());
}

boost::optional<crf::utility::types::JointVelocities> TIMArm::getJointVelocities() {
    logger_->debug("getJointVelocities");
    if (!isInitialized_) {
        logger_->error("The robot hasn't been initialized - Returning false");
        return boost::none;
    }
    //  We consider that the direction defined for each joint in the configuration file is applied
    //  internally on each robot class, that is why we return directly the received velocity.
    auto jointVelocitiesHarmonicArm = etherCATArm_->getJointVelocities();
    if (!jointVelocitiesHarmonicArm) {
        logger_->error("Failed to get the velocity from the EtherCAT robot arm - Returning false");
        return boost::none;
    }
    auto jointVelocitiesKinovaArm = kinovaArm_->getJointVelocities();
    if (!jointVelocitiesKinovaArm) {
        logger_->error("Failed to get the velocity from the Kinova arm - Returning false");
        return boost::none;
    }
    return combineVelocities(jointVelocitiesHarmonicArm.get(), jointVelocitiesKinovaArm.get());
}

boost::optional<crf::utility::types::JointForceTorques> TIMArm::getJointForceTorques() {
    logger_->debug("getJointForceTorques");
    if (!isInitialized_) {
        logger_->error("The robot hasn't been initialized - Returning false");
        return boost::none;
    }
    //  We consider that the direction defined for each joint in the configuration file is applied
    //  internally on each robot class, that is why we return directly the received torque.
    auto jointForceTorquesHarmonicArm = etherCATArm_->getJointForceTorques();
    if (!jointForceTorquesHarmonicArm) {
        logger_->error("Failed to get the torque from the EtherCAT robot arm - Returning false");
        return boost::none;
    }
    auto jointForceTorquesKinovaArm = kinovaArm_->getJointForceTorques();
    if (!jointForceTorquesKinovaArm) {
        logger_->error("Failed to get the torque from the Kinova arm - Returning false");
        return boost::none;
    }
    return combineTorques(jointForceTorquesHarmonicArm.get(), jointForceTorquesKinovaArm.get());
}

boost::optional<crf::utility::types::TaskPose> TIMArm::getTaskPose() {
    logger_->warn("getTaskPose not supported");
    return boost::none;
}

boost::optional<crf::utility::types::TaskVelocity> TIMArm::getTaskVelocity() {
    logger_->warn("getTaskVelocity not supported");
    return boost::none;
}

boost::optional<crf::utility::types::TaskForceTorque> TIMArm::getTaskForceTorque() {
    logger_->warn("getTaskForceTorque not supported");
    return boost::none;
}

bool TIMArm::setJointPositions(const crf::utility::types::JointPositions& jointPositions) {
    logger_->warn("The joints positions can not be set directly for security reasons, since the"\
        "Kinova arm has to move first to home position");
    return false;
}

bool TIMArm::setJointVelocities(const crf::utility::types::JointVelocities& jointVelocities) {
    logger_->debug("setJointVelocities");
    if (!isInitialized_) {
        logger_->error("The robot hasn't been initialized - Returning false");
        return false;
    }
    std::vector<robotarm::JointLimits> limits = armConfiguration_->getJointsConfiguration();
    for (unsigned int i = 0; i < armConfiguration_->getNumberOfJoints(); i++) {
        if (std::fabs(jointVelocities[i]) > limits[i].maximumVelocity) {
            logger_->error("The velocity {} is outside the {} joint limits", jointVelocities[i], i);
            return false;
        }
    }

    //  We consider that the direction defined for each joint in the configuration file is applied
    //  internally on each robot class, that is why we pass directly the input velocity.
    if (!etherCATArm_->setJointVelocities(extractHarmonicVelocity(jointVelocities))) {
        logger_->error("Failed to set the velocity of the EtherCAT robot arm - Returning false");
        return false;
    }
    if (!kinovaArm_->setJointVelocities(extractKinovaVelocity(jointVelocities))) {
        logger_->error("Failed to set the velocity of the Kinova arm - Returning false");
        if (!etherCATArm_->stopArm()) {
            logger_->critical("Failed to cancel movement for the Harmonic Arm");
        }
        return false;
    }
    return true;
}

bool TIMArm::setJointForceTorques(const crf::utility::types::JointForceTorques& jointForceTorques) {
    logger_->debug("setJointForceTorques");
    if (!isInitialized_) {
        logger_->error("The robot hasn't been initialized - Returning false");
        return false;
    }
    std::vector<robotarm::JointLimits> limits = armConfiguration_->getJointsConfiguration();
    for (unsigned int i = 0; i < armConfiguration_->getNumberOfJoints(); i++) {
        if (std::fabs(jointForceTorques[i]) > limits[i].maximumTorque) {
            logger_->error("The torque {} is outside the {} joint limits", jointForceTorques[i], i);
            return false;
        }
    }

    //  We consider that the direction defined for each joint in the configuration file is applied
    //  internally on each robot class, that is why we pass directly the input torque.
    if (!etherCATArm_->setJointForceTorques(extractHarmonicTorque(jointForceTorques))) {
        logger_->error("Failed to set the torque of the EtherCAT robot arm - Returning false");
        return false;
    }
    if (!kinovaArm_->setJointForceTorques(extractKinovaTorque(jointForceTorques))) {
        logger_->error("Failed to set the torque of the Kinova arm - Returning false");
        if (!etherCATArm_->stopArm()) {
            logger_->critical("Failed to cancel movement for the Harmonic Arm");
        }
        return false;
    }
    return true;
}

bool TIMArm::setTaskPose(const crf::utility::types::TaskPose& position) {
    logger_->warn("setTaskPose is not supported");
    return false;
}

bool TIMArm::setTaskVelocity(const crf::utility::types::TaskVelocity& velocity,
    bool TCP) {
    logger_->warn("setTaskVelocity is not supported");
    return false;
}

bool TIMArm::stopArm() {
    logger_->debug("stopArm");
    if (!isInitialized_) {
        logger_->error("The robot hasn't been initialized - Returning false");
        return false;
    }
    if (!etherCATArm_->stopArm()) {
        logger_->error("Failed to stop the EtherCAT robot arm");
        return false;
    }
    if (!kinovaArm_->stopArm()) {
        logger_->error("Failed to stop the Kinova arm");
        return false;
    }
    return true;
}

bool TIMArm::enableBrakes() {
    logger_->debug("enableBrakes");
    if (!isInitialized_) {
        logger_->error("The robot hasn't been initialized - Returning false");
        return false;
    }
    if (!etherCATArm_->enableBrakes()) {
        logger_->error("Failed to enable the breaks of the EtherCAT robot arm");
        return false;
    }
    if (!kinovaArm_->enableBrakes()) {
        logger_->error("Failed to enable the breaks of the Kinova arm");
        return false;
    }
    return true;
}

bool TIMArm::disableBrakes() {
    logger_->debug("disableBrakes");
    if (!isInitialized_) {
        logger_->error("The robot hasn't been initialized - Returning false");
        return false;
    }
    if (!etherCATArm_->disableBrakes()) {
        logger_->error("Failed to disable the breaks of the EtherCAT robot arm");
        return false;
    }
    if (!kinovaArm_->disableBrakes()) {
        logger_->error("Failed to disable the breaks of the Kinova arm");
        return false;
    }
    return true;
}

std::shared_ptr<crf::actuators::robotarm::RobotArmConfiguration> TIMArm::getConfiguration() {
    logger_->debug("getConfiguration");
    return armConfiguration_;
}

crf::utility::types::JointPositions TIMArm::combinePositions(
    const crf::utility::types::JointPositions& jointPositionsHarmonicArm,
    const crf::utility::types::JointPositions& jointPositionsKinovaArm) {
    logger_->debug("combinePositions");
    crf::utility::types::JointPositions result(armConfiguration_->getNumberOfJoints());
    unsigned int etherCATNumberOfJoints = armConfiguration_->getHarmonicNumberOfJoints();
    for (unsigned int etherCATIndex=0; etherCATIndex < etherCATNumberOfJoints; etherCATIndex++) {
        result[etherCATIndex] = jointPositionsHarmonicArm[etherCATIndex];
    }
    unsigned int kinovaJointsNumber = armConfiguration_->getKinovaNumberOfJoints();
    for (unsigned int kinovaIndex=0; kinovaIndex < kinovaJointsNumber; kinovaIndex++) {
        result[etherCATNumberOfJoints + kinovaIndex] = jointPositionsKinovaArm[kinovaIndex];
    }
    return result;
}

crf::utility::types::JointVelocities TIMArm::combineVelocities(
    const crf::utility::types::JointVelocities& jointVelocitiesHarmonicArm,
    const crf::utility::types::JointVelocities& jointVelocitiesKinovaArm) {
    logger_->debug("combineVelocities");
    crf::utility::types::JointVelocities result(armConfiguration_->getNumberOfJoints());
    unsigned int etherCATNumberOfJoints = armConfiguration_->getHarmonicNumberOfJoints();
    for (unsigned int etherCATIndex=0; etherCATIndex < etherCATNumberOfJoints; etherCATIndex++) {
        result[etherCATIndex] = jointVelocitiesHarmonicArm[etherCATIndex];
    }
    unsigned int kinovaJointsNumber = armConfiguration_->getKinovaNumberOfJoints();
    for (unsigned int kinovaIndex=0; kinovaIndex < kinovaJointsNumber; kinovaIndex++) {
        result[etherCATNumberOfJoints + kinovaIndex] = jointVelocitiesKinovaArm[kinovaIndex];
    }
    return result;
}

crf::utility::types::JointForceTorques TIMArm::combineTorques(
    const crf::utility::types::JointForceTorques& jointForceTorquesHarmonicArm,
    const crf::utility::types::JointForceTorques& jointForceTorquesKinovaArm) {
    logger_->debug("combineTorques");
    crf::utility::types::JointForceTorques result(armConfiguration_->getNumberOfJoints());
    unsigned int etherCATNumberOfJoints = armConfiguration_->getHarmonicNumberOfJoints();
    for (unsigned int etherCATIndex=0; etherCATIndex < etherCATNumberOfJoints; etherCATIndex++) {
        result[etherCATIndex] = jointForceTorquesHarmonicArm[etherCATIndex];
    }
    unsigned int kinovaJointsNumber = armConfiguration_->getKinovaNumberOfJoints();
    for (unsigned int kinovaIndex=0; kinovaIndex < kinovaJointsNumber; kinovaIndex++) {
        result[etherCATNumberOfJoints + kinovaIndex] = jointForceTorquesKinovaArm[kinovaIndex];
    }
    return result;
}

crf::utility::types::JointVelocities TIMArm::extractKinovaVelocity(
    const crf::utility::types::JointVelocities& jointVelocitiesTIMArm) {
    logger_->debug("extractKinovaVelocity");
    unsigned int etherCATNumberOfJoints = armConfiguration_->getHarmonicNumberOfJoints();
    unsigned int kinovaJointsNumber = armConfiguration_->getKinovaNumberOfJoints();
    crf::utility::types::JointVelocities result(kinovaJointsNumber);
    for (unsigned int kinovaIndex=0; kinovaIndex < kinovaJointsNumber; kinovaIndex++) {
        result[kinovaIndex] = jointVelocitiesTIMArm[etherCATNumberOfJoints + kinovaIndex];
    }
    return result;
}

crf::utility::types::JointVelocities TIMArm::extractHarmonicVelocity(
    const crf::utility::types::JointVelocities& jointVelocitiesTIMArm) {
    logger_->debug("extractHarmonicVelocity");
    unsigned int etherCATNumberOfJoints = armConfiguration_->getHarmonicNumberOfJoints();
    crf::utility::types::JointVelocities result(etherCATNumberOfJoints);
    for (unsigned int etherCATIndex=0; etherCATIndex < etherCATNumberOfJoints; etherCATIndex++) {
        result[etherCATIndex] = jointVelocitiesTIMArm[etherCATIndex];
    }
    return result;
}

crf::utility::types::JointForceTorques TIMArm::extractKinovaTorque(
    const crf::utility::types::JointForceTorques& jointForceTorquesTIMArm) {
    logger_->debug("extractKinovaTorque");
    unsigned int etherCATNumberOfJoints = armConfiguration_->getHarmonicNumberOfJoints();
    unsigned int kinovaJointsNumber = armConfiguration_->getKinovaNumberOfJoints();
    crf::utility::types::JointForceTorques result(kinovaJointsNumber);
    for (unsigned int kinovaIndex=0; kinovaIndex < kinovaJointsNumber; kinovaIndex++) {
        result[kinovaIndex] = jointForceTorquesTIMArm[etherCATNumberOfJoints + kinovaIndex];
    }
    return result;
}

crf::utility::types::JointForceTorques TIMArm::extractHarmonicTorque(
    const crf::utility::types::JointForceTorques& jointForceTorquesTIMArm) {
    logger_->debug("extractHarmonicTorque");
    unsigned int etherCATNumberOfJoints = armConfiguration_->getHarmonicNumberOfJoints();
    crf::utility::types::JointForceTorques result(etherCATNumberOfJoints);
    for (unsigned int etherCATIndex=0; etherCATIndex < etherCATNumberOfJoints; etherCATIndex++) {
        result[etherCATIndex] = jointForceTorquesTIMArm[etherCATIndex];
    }
    return result;
}

}  // namespace crf::actuators::timarm
