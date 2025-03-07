/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <thread>

#include <RaptorAPI.hpp>

#include "Haption/IHaptionAPI.hpp"
#include "EventLogger/EventLogger.hpp"
#include "crf/ResponseDefinitions.hpp"

namespace crf::devices::haption {

/**
 * @brief Class to communicate with the Haption devices using the Raptor API. This class already
 *        handles internally the communication loop with the device.
 */
class HaptionRaptorAPI: public IHaptionAPI {
 public:
    /**
     * @brief Construct a new Haption Raptor API object in the case of an Ethernet-based device.
     * 
     * @param localPort Any free UDP port of the local PC.
     * @param remoteIP IP of the Haption device.
     * @param remotePort Port of the Haption device. it must be set to 5000
     * @param parametersFile Parameters file of the Haption Device
     */
    HaptionRaptorAPI(unsigned int localPort, const std::string& remoteIP, unsigned int remotePort,
        const std::string& parametersFile);
    /**
     * @brief Construct a new Haption Raptor API object in the case of an EtherCAT-based device
     *        connected to a Hilscher CIFX master card.
     * 
     * @param card CIFX card number, typically 0.
     * @param slot CIFX card slot setting, typically 1.
     * @param offset offset on the bus, typically 0.
     * @param parametersFile Parameters file of the Haption Device.
     */
    HaptionRaptorAPI(unsigned int card, unsigned int slot, unsigned int offset,
        const std::string& parametersFile);
    ~HaptionRaptorAPI() override;

    crf::Code startConnection() override;
    crf::Code stopConnection() override;
    crf::Code calibrate() override;

    crf::Code activateForceFeedback(const bool& iActivate) override;

    crf::Code getJointAngles(HAPTION::JointVector& oAngles) override;
    crf::Code getJointSpeeds(HAPTION::JointVector& oSpeeds) override;
    crf::Code getJointTorques(HAPTION::JointVector& oTorques) override;
    crf::Code getMotorCurrents(HAPTION::JointVector& oCurrents) override;

    crf::Code getCartesianPose(HAPTION::Displacement& oPose) override;
    crf::Code getCartesianPose(HAPTION::Transformation& oPose) override;
    crf::Code getRawCartesianPose(HAPTION::Displacement& oPose) override;
    crf::Code getRawCartesianPose(HAPTION::Transformation& oPose) override;
    crf::Code getCartesianSpeed(HAPTION::CartesianVector& oSpeed) override;
    crf::Code getCartesianForce(HAPTION::CartesianVector& oForce) override;

    crf::Code startJointPositionMode() override;
    crf::Code setJointAngles(const HAPTION::JointVector& iJointAngles) override;
    crf::Code setJointSpeeds(const HAPTION::JointVector& iJointSpeeds) override;
    crf::Code addJointTorqueOverlay(const HAPTION::JointVector& iJointTorques) override;

    crf::Code startCartesianPositionMode() override;
    crf::Code setCartesianPose(const HAPTION::Displacement& iPose) override;
    crf::Code setCartesianPose(const HAPTION::Transformation& iPose) override;
    crf::Code setCartesianPoseWithClutchOffset(const HAPTION::Displacement& iPose,
        const HAPTION::Displacement& iOffset) override;
    crf::Code setCartesianSpeed(const HAPTION::CartesianVector& iSpeed) override;
    crf::Code addCartesianForceOverlay(const HAPTION::CartesianVector& iForce) override;

    crf::Code activateClutch(bool iActivation) override;
    crf::Code getClutchOffset(HAPTION::Displacement& oOffset) override;
    crf::Code getClutchOffset(HAPTION::Transformation& oOffset) override;

    crf::Code changeBrakeStatus(const HAPTION::BrakeStatus& iStatus) override;
    crf::Code getBrakeStatus(HAPTION::BrakeStatus& oStatus) override;

    crf::Code getErrorStatus() override;
    crf::Code getPowerStatus(HAPTION::PowerStatus& oPowerStatus) override;
    crf::Code getAutomatonStatus(HAPTION::AutomatonStatus& oStatus) override;
    crf::Code getMotorStatus(
        std::array<HAPTION::MotorStatus, HAPTION::MAX_NB_JOINTS> &oStatus) override;

    crf::Code clearError() override;

    crf::Code changeBase(const HAPTION::Displacement& iDisp) override;
    crf::Code changeBase(const HAPTION::Transformation& iTransformation) override;
    crf::Code changeViewpoint(const HAPTION::Displacement& iDisp) override;
    crf::Code changeViewpoint(const HAPTION::Transformation& iTransformation) override;
    crf::Code moveViewpoint(const HAPTION::Displacement& iDisp,
        const HAPTION::CartesianVector& iSpeed) override;
    crf::Code moveViewpoint(const HAPTION::Transformation& iTransformation,
        const HAPTION::CartesianVector& iSpeed) override;
    crf::Code changeMovementScale(const HAPTION::float32_t& iScaleTrans,
        const HAPTION::float32_t& iScaleRot) override;

    crf::Code changeCartesianGains(HAPTION::float32_t iKT, HAPTION::float32_t iBT,
        HAPTION::float32_t iKR, HAPTION::float32_t iBR) override;
    crf::Code getMaxCartesianGains(HAPTION::float32_t& oMaxKT, HAPTION::float32_t& oMaxBT,
        HAPTION::float32_t& oMaxKR, HAPTION::float32_t& oMaxBR) override;

    crf::Code changeJointGains(const HAPTION::JointVector& iKs,
        const HAPTION::JointVector& iBs) override;
    crf::Code getMaxJointGains(HAPTION::JointVector& oMaxKs,
        HAPTION::JointVector& oMaxBs) override;

    crf::Code forceCartesianPose(const HAPTION::Displacement& iPose) override;
    crf::Code forceCartesianPose(const HAPTION::Transformation& iPose) override;

    crf::Code getOperatorButton(const HAPTION::OperatorButton& iButton,
        bool& oButtonState) override;
    crf::Code getTriggerValue(HAPTION::float32_t& oValue) override;

    crf::Code switchPowerOnOff(const bool& iEnableSupply) override;
    crf::Code getTCPOffset(HAPTION::Displacement& oTcpOffset) override;
    crf::Code changeTCPOffset(const HAPTION::Displacement& iTcpOffset) override;
    crf::Code setTriggerLockLed(const bool& iStatus) override;
    crf::Code isJointLimitReached(std::array<bool, HAPTION::MAX_NB_JOINTS>& oLimitsReached) override;  // NOLINT

 private:
    std::string parametersFile_;
    crf::utility::logger::EventLogger logger_;
    HAPTION::RaptorAPI raptorHandle_;
    bool connected_;
    unsigned int calibTimeoutSec_;
    bool communicationLoopStopped_;
    std::thread communicationLoopThread_;
    const std::chrono::milliseconds loopTime_ = std::chrono::milliseconds(1);

    void communicationLoop();
    crf::Code errorParser(HAPTION::ErrorCode code);
};

}  // namespace crf::devices::haption
