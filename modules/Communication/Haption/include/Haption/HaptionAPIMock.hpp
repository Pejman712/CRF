/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>
#include <string>
#include <array>

#include "Haption/IHaptionAPI.hpp"

namespace crf::devices::haption {

class HaptionAPIMock : public IHaptionAPI {
 public:
    MOCK_METHOD(crf::Code, startConnection, (), (override));
    MOCK_METHOD(crf::Code, stopConnection, (), (override));
    MOCK_METHOD(crf::Code, calibrate, (), (override));
    MOCK_METHOD(crf::Code, activateForceFeedback, (const bool& iActivate), (override));
    MOCK_METHOD(crf::Code, getJointAngles, (HAPTION::JointVector& oAngles), (override));  // NOLINT
    MOCK_METHOD(crf::Code, getJointSpeeds, (HAPTION::JointVector& oSpeeds), (override));  // NOLINT
    MOCK_METHOD(crf::Code, getJointTorques, (HAPTION::JointVector& oTorques), (override));  // NOLINT
    MOCK_METHOD(crf::Code, getMotorCurrents, (HAPTION::JointVector& oCurrents), (override));  // NOLINT
    MOCK_METHOD(crf::Code, getCartesianPose, (HAPTION::Displacement& oPose), (override));  // NOLINT
    MOCK_METHOD(crf::Code, getCartesianPose, (HAPTION::Transformation& oPose), (override));  // NOLINT
    MOCK_METHOD(crf::Code, getRawCartesianPose, (HAPTION::Displacement& oPose), (override));  // NOLINT
    MOCK_METHOD(crf::Code, getRawCartesianPose, (HAPTION::Transformation& oPose), (override));  // NOLINT
    MOCK_METHOD(crf::Code, getCartesianSpeed, (HAPTION::CartesianVector& oSpeed), (override));  // NOLINT
    MOCK_METHOD(crf::Code, getCartesianForce, (HAPTION::CartesianVector& oForce), (override));  // NOLINT
    MOCK_METHOD(crf::Code, startJointPositionMode, (), (override));
    MOCK_METHOD(crf::Code, setJointAngles, (const HAPTION::JointVector& iJointAngles), (override));
    MOCK_METHOD(crf::Code, setJointSpeeds, (const HAPTION::JointVector& iJointSpeeds), (override));
    MOCK_METHOD(crf::Code, addJointTorqueOverlay, (const HAPTION::JointVector& iJointTorques),
        (override));
    MOCK_METHOD(crf::Code, startCartesianPositionMode, (), (override));
    MOCK_METHOD(crf::Code, setCartesianPose, (const HAPTION::Displacement& iPose), (override));
    MOCK_METHOD(crf::Code, setCartesianPose, (const HAPTION::Transformation& iPose), (override));
    MOCK_METHOD(crf::Code, setCartesianPoseWithClutchOffset,
        (const HAPTION::Displacement& iPose, const HAPTION::Displacement& iOffset), (override));
    MOCK_METHOD(crf::Code, setCartesianSpeed, (const HAPTION::CartesianVector& iSpeed),
        (override));
    MOCK_METHOD(crf::Code, addCartesianForceOverlay, (const HAPTION::CartesianVector& iForce),
        (override));
    MOCK_METHOD(crf::Code, activateClutch, (bool iActivation), (override));
    MOCK_METHOD(crf::Code, getClutchOffset, (HAPTION::Displacement& oOffset), (override));  // NOLINT
    MOCK_METHOD(crf::Code, getClutchOffset, (HAPTION::Transformation& oOffset), (override));  // NOLINT
    MOCK_METHOD(crf::Code, changeBrakeStatus, (const HAPTION::BrakeStatus& iStatus), (override));
    MOCK_METHOD(crf::Code, getBrakeStatus, (HAPTION::BrakeStatus& oStatus), (override));  // NOLINT
    MOCK_METHOD(crf::Code, getErrorStatus, (), (override));
    MOCK_METHOD(crf::Code, getPowerStatus, (HAPTION::PowerStatus& oPowerStatus), (override));  // NOLINT
    MOCK_METHOD(crf::Code, getAutomatonStatus, (HAPTION::AutomatonStatus& oStatus), (override));  // NOLINT
    MOCK_METHOD(crf::Code, getMotorStatus, ((std::array<HAPTION::MotorStatus,
        HAPTION::MAX_NB_JOINTS>& oStatus)), (override));  // NOLINT
    MOCK_METHOD(crf::Code, clearError, (), (override));
    MOCK_METHOD(crf::Code, changeBase, (const HAPTION::Displacement& iDisp), (override));
    MOCK_METHOD(crf::Code, changeBase, (const HAPTION::Transformation& iTransformation),
        (override));
    MOCK_METHOD(crf::Code, changeViewpoint, (const HAPTION::Displacement& iDisp), (override));
    MOCK_METHOD(crf::Code, changeViewpoint, (const HAPTION::Transformation& iTransformation),
        (override));
    MOCK_METHOD(crf::Code, moveViewpoint, (const HAPTION::Displacement& iDisp,
        const HAPTION::CartesianVector& iSpeed), (override));
    MOCK_METHOD(crf::Code, moveViewpoint, (const HAPTION::Transformation& iTransformation,
        const HAPTION::CartesianVector& iSpeed), (override));
    MOCK_METHOD(crf::Code, changeMovementScale, (const HAPTION::float32_t& iScaleTrans,
        const HAPTION::float32_t& iScaleRot), (override));
    MOCK_METHOD(crf::Code, changeCartesianGains, (HAPTION::float32_t iKT, HAPTION::float32_t iBT,
        HAPTION::float32_t iKR, HAPTION::float32_t iBR), (override));
    MOCK_METHOD(crf::Code, getMaxCartesianGains, (HAPTION::float32_t& oMaxKT,  // NOLINT
        HAPTION::float32_t& oMaxBT, HAPTION::float32_t& oMaxKR, HAPTION::float32_t& oMaxBR),  // NOLINT
        (override));
    MOCK_METHOD(crf::Code, changeJointGains, (const HAPTION::JointVector& iKs,
        const HAPTION::JointVector& iBs), (override));
    MOCK_METHOD(crf::Code, getMaxJointGains, (HAPTION::JointVector& oMaxKs,  // NOLINT
        HAPTION::JointVector& oMaxBs), (override));  // NOLINT
    MOCK_METHOD(crf::Code, forceCartesianPose, (const HAPTION::Displacement& iPose), (override));
    MOCK_METHOD(crf::Code, forceCartesianPose, (const HAPTION::Transformation& iPose), (override));
    MOCK_METHOD(crf::Code, getOperatorButton, (const HAPTION::OperatorButton& iButton,
        bool& oButtonState), (override));  // NOLINT
    MOCK_METHOD(crf::Code, getTriggerValue, (HAPTION::float32_t& oValue), (override));  // NOLINT
    MOCK_METHOD(crf::Code, switchPowerOnOff, (const bool& iEnableSupply), (override));
    MOCK_METHOD(crf::Code, getTCPOffset, (HAPTION::Displacement& oTcpOffset), (override));  // NOLINT
    MOCK_METHOD(crf::Code, changeTCPOffset, (const HAPTION::Displacement& iTcpOffset), (override));
    MOCK_METHOD(crf::Code, setTriggerLockLed, (const bool& iStatus), (override));  // NOLINT
    MOCK_METHOD(crf::Code, isJointLimitReached, ((std::array<bool,
        HAPTION::MAX_NB_JOINTS>& oLimitsReached)), (override));  // NOLINT
};

}  // namespace crf::devices::haption
