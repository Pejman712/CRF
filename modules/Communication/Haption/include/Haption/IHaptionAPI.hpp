/* © Copyright CERN 2024. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN BE/CEM/MRO 2024
 *
 *  ==================================================================================================
 */

#pragma once

#include <RaptorAPI.hpp>

#include "crf/ResponseDefinitions.hpp"

namespace crf::devices::haption {

/**
 * @brief Missing StartLogging() and StopLogging()
*/
class IHaptionAPI {
 public:
    virtual ~IHaptionAPI() = default;

    /**
     * @brief Connects to the haption device and triggers the calibration if needed. It launches
     *        the communication thread to write and read from the device.
     * @return crf::Code 
     */
    virtual crf::Code startConnection() = 0;
    /**
     * @brief Stops the communication thread with the haption device and closes the connection.
     * @return crf::Code 
     */
    virtual crf::Code stopConnection() = 0;
    /**
     * @brief It triggers the calibration process with instructions to the user.
     * @return crf::Code 
     */
    virtual crf::Code calibrate() = 0;

    /**
     * @brief Get the error status of the device. A call to ReadState() will refresh the error
     *        status.
     * @return crf::Code 
     */
    virtual crf::Code getErrorStatus() = 0;
    /**
     * @brief Set the base transformation. This is available only after a successful call to Init()
     *        and before any "Start" function has been called.
     * @param[in] iDisp Transformation from viewpoint frame to base frame.
     * @return crf::Code 
     */
    virtual crf::Code changeBase(HAPTION::Displacement const& iDisp) = 0;
    /**
     * @brief Set the base transformation. This is available only after a successful call to Init()
     *        and before any "Start" function has been called.
     * @param[in] iTransformation Transformation matrix from viewpoint frame to base frame.
     * @return crf::Code 
     */
    virtual crf::Code changeBase(HAPTION::Transformation const& iTransformation) = 0;
    /**
     * @brief Set the viewpoint transformation. This is available at any time between Init() and
     *        Close(). This call modifies the clutch offset.
     * @param[in] iDisp Transformation from world frame to viewpoint frame.
     * @return crf::Code
     */
    virtual crf::Code changeViewpoint(HAPTION::Displacement const& iDisp) = 0;
    /**
     * @brief Set the viewpoint transformation. This is available at any time between Init() and
     *        Close(). This call modifies the clutch offset.
     * @param[in] iTransformation Transformation matrix from world frame to viewpoint frame.
     * @return crf::Code.
     */
    virtual crf::Code changeViewpoint(HAPTION::Transformation const& iTransformation) = 0;
    /**
     * @brief Change continuously the viewpoint transformation (this might generate forces!). This
     *        is available at any time between Init() and Close(). This call modifies the clutch
     *        offset.
     * @param[in] iDisp Transformation from world frame to viewpoint frame.
     * @param[in] iSpeed Motion from world frame to viewpoint frame.
     * @return crf::Code
     */
    virtual crf::Code moveViewpoint(HAPTION::Displacement const& iDisp,
        HAPTION::CartesianVector const& iSpeed) = 0;
    /**
     * @brief Change continuously the viewpoint frame (this might generate forces!). This is
     *        available at any time between Init() and Close(). This call modifies the clutch
     *        offset.
     * @param[in] iTransformation Transformation from world frame to viewpoint frame.
     * @param[in] iSpeed Motion from world frame to viewpoint frame.
     * @return crf::Code
     */
    virtual crf::Code moveViewpoint(HAPTION::Transformation const& iTransformation,
        HAPTION::CartesianVector const& iSpeed) = 0;
    /**
     * @brief Change movement scaling. This is available at any time between Init() and Close().
     *        This call modifies the clutch offset.
     * @param[in] iScaleTrans Scaling of translations.
     * @param[in] iScaleRot Scaling of rotations.
     * @return crf::Code
     */
    virtual crf::Code changeMovementScale(const HAPTION::float32_t& iScaleTrans,
        const HAPTION::float32_t& iScaleRot) = 0;
    /**
     * @brief Get the current (relative) Cartesian pose. This is available after a successful call
     *        to Init(). A call to ReadState() is needed to refresh the data before calling this
     *        function.
     * @param[out] oPose Current Cartesian pose in the world frame.
     * @return crf::Code
     */
    virtual crf::Code getCartesianPose(HAPTION::Displacement& oPose) = 0;  // NOLINT
    /**
     * @brief Get the current (relative) Cartesian pose. This is available after a successful call
     *        to Init(). A call to ReadState() is needed to refresh the data before calling this
     *        function.
     * @param[out] oPose Current Cartesian pose in the world frame.
     * @return crf::Code
     */
    virtual crf::Code getCartesianPose(HAPTION::Transformation& oPose) = 0;  // NOLINT
    /**
     * @brief Get the current (absolute) Cartesian pose. This function returns the current absolute
     *        cartesian pose in the base frame without including the clutch offset, movement scale,
     *        base orientation, and observation frame. This is available after a successful call to
     *        Init(). A call to ReadState() is needed to refresh the data before calling this
     *        function.
     * @param[out] oPose Current Cartesian pose in the base frame
     * @return crf::Code
     */
    virtual crf::Code getRawCartesianPose(HAPTION::Displacement& oPose) = 0;  // NOLINT
    /**
     * @brief Get the current (absolute) Cartesian pose. This function returns the current absolute
     *        cartesian pose in the base frame without including the clutch offset, movement scale,
     *        base orientation, and observation frame. This is available after a successful call to
     *        Init(). A call to ReadState() is needed to refresh the data before calling this
     *        function.
     * @param[out] oPose Current Cartesian pose in the base frame
     * @return crf::Code
     */
    virtual crf::Code getRawCartesianPose(HAPTION::Transformation& oPose) = 0;  // NOLINT
    /**
     * @brief Get the current (relative) Cartesian speed. This is available after a successful call
     *        to Init(). A call to ReadState() is needed to refresh the data before calling this
     *        function.
     * @param[out] oSpeed Current Cartesian speed in the world frame.
     * @return crf::Code
     */
    virtual crf::Code getCartesianSpeed(HAPTION::CartesianVector& oSpeed) = 0;  // NOLINT
    /**
     * @brief Get the current joint angles. This is available after a successful call to Init(). A
     *        call to ReadState() is needed to refresh the data before calling this function.
     * @param[out] oAngles Current joint angles.
     * @return crf::Code
     */
    virtual crf::Code getJointAngles(HAPTION::JointVector& oAngles) = 0;  // NOLINT
    /**
     * @brief Get the current joint speeds. This is available after a successful call to Init(). A
     *        call to ReadState() is needed to refresh the data before calling this function.
     * @param[out] oSpeeds Current joint rotation speeds.
     * @return crf::Code
     */
    virtual crf::Code getJointSpeeds(HAPTION::JointVector& oSpeeds) = 0;  // NOLINT
    /**
     * @brief Get the current measured joint torques. This function calculates the joints torques
     *        from raw motors currents. This is available after a successful call to Init(). A call
     *        to ReadState() is needed to refresh the data before calling this function.
     * @param[out] oTorques Current joint torques.
     * @return crf::Code
     */
    virtual crf::Code getJointTorques(HAPTION::JointVector& oTorques) = 0;  // NOLINT
    /**
     * @brief Get the current measured motors current. This is available after a successful call to
     *        Init(). A call to ReadState() is needed to refresh the data before calling this
     *        function.
     * @param[out] oCurrents Current motors current.
     * @return crf::Code
     * @note Not available on all devices (Achille-specific)
     */
    virtual crf::Code getMotorCurrents(HAPTION::JointVector& oCurrents) = 0;  // NOLINT
    /**
     * @brief Get the detailed status of the motors. This is available after a successful call to
     *        Init(). A call to ReadState() is needed to refresh the data before calling this
     *        function.
     * @param[out] oStatus Current motors status.
     * @return crf::Code
     * @note Not available on all devices (Achille-specific).
     */
    virtual crf::Code getMotorStatus(std::array<HAPTION::MotorStatus, HAPTION::MAX_NB_JOINTS> &oStatus) = 0;  // NOLINT
    /**
     * @brief Change the Cartesian gains. This is available after a successful call to
     *        StartCartesianPositionMode().
     * @param[in] iKT Proportional gain on translation, in N/m
     * @param[in] iBT Derivate gain on translation, in N/(m/s)
     * @param[in] iKR Proportional gain on rotation, in Nm/rad
     * @param[in] iBR Derivate gain on rotation, in Nm/(rad/s)
     * @return crf::Code
     */
    virtual crf::Code changeCartesianGains(HAPTION::float32_t iKT, HAPTION::float32_t iBT,  // NOLINT
        HAPTION::float32_t iKR, HAPTION::float32_t iBR) = 0;  // NOLINT
    /**
     * @brief Get the maximum value of the cartesian gains. This is available after a successful
     *        call to StartCartesianPositionMode().
     * @param[out] oMaxKT Maximum proportional gain on translation, in N/m
     * @param[out] oMaxBT Maximum derivate gain on translation, in N/(m/s)
     * @param[out] oMaxKR Maximum proportional gain on rotation, in Nm/rad
     * @param[out] oMaxBR Maximum derivate gain on rotation, in Nm/(rad/s)
     * @return crf::Code
     */
    virtual crf::Code getMaxCartesianGains(HAPTION::float32_t& oMaxKT,  // NOLINT
        HAPTION::float32_t& oMaxBT, HAPTION::float32_t& oMaxKR, HAPTION::float32_t& oMaxBR) = 0;  // NOLINT
    /**
     * @brief Start Cartesian position control mode.This is available after a successful call to
     *        Init().
     * @return crf::Code
     */
    virtual crf::Code startCartesianPositionMode() = 0;
    /**
     * @brief Change the joint-space gains. This is available after a successful call to
     *        StartJointPositionMode().
     * @param[in] iKs Proportional gains for all joints, in Nm/rad
     * @param[in] iBs Derivate gains for all joints, in Nm/(rad/s)
     * @return crf::Code
     */
    virtual crf::Code changeJointGains(HAPTION::JointVector const& iKs,
        HAPTION::JointVector const& iBs) = 0;
    /**
     * @brief Get the maximum value of the joint gains. This is available after a successful call
     *        to StartJointPositionMode().
     * @param[out] oMaxKs Maximum proportional gains for all joints, in Nm/rad
     * @param[out] oMaxBs Maximum derivate gains for all joints, in Nm/(rad/s)
     * @return crf::Code
     */
    virtual crf::Code getMaxJointGains(HAPTION::JointVector& oMaxKs,  // NOLINT
        HAPTION::JointVector& oMaxBs) = 0;  // NOLINT
    /**
     * @brief Start joint position control mode. This is available after a successful call to
     *        Init().
     * @return crf::Code
     */
    virtual crf::Code startJointPositionMode() = 0;
    /**
     * @brief Activate or deactivate force-feedback. This is available after a successful call to
     *        Init().
     * @param[in] iActivate True to activate force-feedback, false to deactivate.
     * @return crf::Code
     */
    virtual crf::Code activateForceFeedback(const bool& iActivate) = 0;
    /**
     * @brief Engage or release the brakes. This is available after a successful call to Init().
     * @param[in] iStatus Status command,
     * @return crf::Code
     * @note Not available on all devices (Achille-specific)
     */
    virtual crf::Code changeBrakeStatus(const HAPTION::BrakeStatus& iStatus) = 0;
    /**
     * @brief Get the current status of the brakes, whether engaged or released. This is available
     *        after a successful call to Init().
     * @param[out] oStatus Status command
     * @return crf::Code
     * @note Not available on all devices (Achille-specific)
     */
    virtual crf::Code getBrakeStatus(HAPTION::BrakeStatus& oStatus) = 0;  // NOLINT
    /**
     * @brief Get the status of the automaton. This is available after a successful call to Init().
     * @param[out] oStatus Current automaton status.
     * @return crf::Code
     */
    virtual crf::Code getAutomatonStatus(HAPTION::AutomatonStatus& oStatus) = 0;  // NOLINT
    /**
     * @brief Get the status of motor power. This is available after a successful call to Init().
     * @param[out] oPowerStatus Current power status.
     * @return crf::Code
     */
    virtual crf::Code getPowerStatus(HAPTION::PowerStatus& oPowerStatus) = 0;  // NOLINT
    /**
     * @brief Force the current Cartesian pose, overwriting the clutch offset. This is available
     *        after a successful call to StartCartesianPositionMode(). This call modifies the
     *        clutch offset.
     * @param[in] iPose Cartesian pose to be returned by the next call to GetCartesianPose.
     * @return crf::Code
     */
    virtual crf::Code forceCartesianPose(HAPTION::Displacement const& iPose) = 0;
    /**
     * @brief Force the current Cartesian pose, overwriting the clutch offset. This is available
     *        after a successful call to StartCartesianPositionMode(). This call modifies the
     *        clutch offset.
     * @param[in] iPose Cartesian pose to be returned by the next call to GetCartesianPose.
     * @return crf::Code
     */
    virtual crf::Code forceCartesianPose(HAPTION::Transformation const& iPose) = 0;
    /**
     * @brief Get the current clutch offset. This is available after a successful call to
     *        StartCartesianPositionMode().
     * @param[out] oOffset Current clutch offset in the base frame.
     * @return crf::Code
     */
    virtual crf::Code getClutchOffset(HAPTION::Displacement& oOffset) = 0;  // NOLINT
    /**
     * @brief Activate the clutch by software. This is available after a successful call to
     *        StartCartesianPositionMode().
     * @param[in] iActivation True to activate, False to deactivate.
     * @return crf::Code
     */
    virtual crf::Code activateClutch(bool iActivation) = 0;
    /**
     * @brief Get the current clutch offset. This is available after a successful call to
     *        StartCartesianPositionMode().
     * @param[out] oOffset Current clutch offset in the base frame.
     * @return crf::Code
     */
    virtual crf::Code getClutchOffset(HAPTION::Transformation& oOffset) = 0;  // NOLINT
    /**
     * @brief Set a Cartesian pose setpoint. This is available after a successful call to
     *        StartCartesianPositionMode(). A call to SendSetpoints() is needed for the data to be
     *        effectively sent to the device.
     * @param[in] iPose Cartesian pose setpoint in the world frame.
     * @return crf::Code
     */
    virtual crf::Code setCartesianPose(HAPTION::Displacement const& iPose) = 0;
    /**
     * @brief Set a Cartesian pose setpoint. This is available after a successful call to
     *        StartCartesianPositionMode(). A call to SendSetpoints() is needed for the data to be
     *        effectively sent to the device.
     * @param[in] iPose Cartesian pose setpoint in the world frame.
     * @return crf::Code
     */
    virtual crf::Code setCartesianPose(HAPTION::Transformation const& iPose) = 0;
    /**
     * @brief Set a Cartesian pose setpoint and update the clutch offset.
     * This is available after a successful call to StartCartesianPositionMode().
     * A call to SendSetpoints() is needed for the data to be effectively sent to the device.
     * @param[in] iPose Cartesian pose setpoint in the world frame
     * @param[in] iOffset Cartesian clutch offset to be added in the world frame
     * @return crf::Code
     */
    virtual crf::Code setCartesianPoseWithClutchOffset(HAPTION::Displacement const& iPose,
        HAPTION::Displacement const& iOffset) = 0;
    /**
     * @brief Set a Cartesian speed setpoint. This is available after a successful call to
     *        StartCartesianPositionMode(). A call to SendSetpoints() is needed for the data to be
     *        effectively sent to the device.
     * @param[in] iSpeed Cartesian speed setpoint in the world frame.
     * @return crf::Code
     */
    virtual crf::Code setCartesianSpeed(HAPTION::CartesianVector const& iSpeed) = 0;
    /**
     * @brief Add a Cartesian force overlay. This is available after a successful call to
     *        StartCartesianPositionMode(). A call to SendSetpoints() is needed for the data to be
     *        effectively sent to the device.
     * @param[in] iForce Cartesian force in the world frame.
     * @return crf::Code
     */
    virtual crf::Code addCartesianForceOverlay(HAPTION::CartesianVector const& iForce) = 0;
    /**
     * @brief Get the current Cartesian force applied by the device on the operator's hand. This is
     *        available after a successful call to StartCartesianPositionMode(). A call to
     *        ReadState() is needed to refresh the data.
     * @param[out] oForce Current Cartesian force in the world frame
     * @return crf::Code
     */
    virtual crf::Code getCartesianForce(HAPTION::CartesianVector& oForce) = 0;  // NOLINT
    /**
     * @brief Set a joint angle setpoint.
     * This is available after a successful call to StartJointPositionMode().
     * A call to SendSetpoints() is needed for the data to be effectively sent to the device.
     * @param[in] iJointAngles Joint angles setpoint
     * @return crf::Code
     */
    virtual crf::Code setJointAngles(HAPTION::JointVector const& iJointAngles) = 0;
    /**
     * @brief Set a joint speed setpoint. This is available after a successful call to
     *        StartJointPositionMode(). A call to SendSetpoints() is needed for the data to be
     *        effectively sent to the device.
     * @param[in] iJointSpeeds Joint speeds setpoint.
     * @return crf::Code
     */
    virtual crf::Code setJointSpeeds(HAPTION::JointVector const& iJointSpeeds) = 0;
    /**
     * @brief Add a joint torque overlay. This is available after a successful call to
     *        StartJointPositionMode(). A call to SendSetpoints() is needed for the data to be
     *        effectively sent to the device.
     * @param[in] iJointTorques Joint torque overlay.
     * @return crf::Code
     */
    virtual crf::Code addJointTorqueOverlay(HAPTION::JointVector const& iJointTorques) = 0;
    /**
     * @brief Get the status of an operator button. A call to ReadState() is needed to refresh the
     *        data.
     * @param[in] iButton OperatorButton Identifier of the operator button.
     * @param[out] oButtonState State of the button (true for pressed/active).
     * @return crf::Code
     */
    virtual crf::Code getOperatorButton(const HAPTION::OperatorButton& iButton,
        bool& oButtonState) = 0;  // NOLINT
    /**
     * @brief Get the value of the trigger device. A call to ReadState() is needed to refresh the
     *        data.
     * @param[out] oValue Current value of the trigger, as a float between 0 (open) and 1 (closed).
     * @return crf::Code
     */
    virtual crf::Code getTriggerValue(HAPTION::float32_t& oValue) = 0;  // NOLINT
    /**
     * @brief Clear error in cartesian controller. This is available after a successfull call to
     *        Init().
     * @return crf::Code
     */
    virtual crf::Code clearError() = 0;
    /**
     * @brief Switch the motor power supply on or off. This is available after a successful call to
     *        Init().
     * @param[in] iEnableSupply True to switch On, false to switch Off.
     * @return crf::Code
     */
    virtual crf::Code switchPowerOnOff(const bool& iEnableSupply) = 0;
    /**
     * @brief Get the TCP offset of the robot with respect to the wrist. This is available after a
     *        successful call to Init().
     * @param[out] oTcpOffset TCP offset as a displacement in the end effector frame.
     * @return ErrorCode
     */
    virtual crf::Code getTCPOffset(HAPTION::Displacement& oTcpOffset) = 0;  // NOLINT
    /**
     * @brief 
     * 
     */
    virtual crf::Code changeTCPOffset(const HAPTION::Displacement& iTcpOffset) = 0;
    /**
     * @brief Set the trigger lock led to true or false. This is available after a successful call
     *        to Init(). A call to SendSetpoints() is needed for the data to be effectively sent to
     *        the device.
     * @param[in] iStatus True to turn the led on, false to turn it Off.
     * @return crf::Code
     */
    virtual crf::Code setTriggerLockLed(const bool& iStatus) = 0;
    /**
     * @brief Check if the robot has reached its joints limits. This is available after a
     *        successful call to Init(). A call to ReadState() is needed to refresh the data before
     *        calling this function.
     * @param[out] oLimitsReached Flags showing which joints have reached their limits.
     * @return crf::Code
     */
    virtual crf::Code isJointLimitReached(
        std::array<bool, HAPTION::MAX_NB_JOINTS>& oLimitsReached) = 0;  // NOLINT
};

}  // namespace crf::devices::haption
