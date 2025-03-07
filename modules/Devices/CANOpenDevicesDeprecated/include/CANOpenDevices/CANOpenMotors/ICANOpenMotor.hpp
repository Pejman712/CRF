/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <memory>
#include <utility>
#include <optional>

#include "CANOpenDevices/ICANOpenDevice.hpp"
#include "CANOpenDevices/ObjectDictionary.hpp"

namespace crf {
namespace devices {
namespace canopendevices {

class ICANOpenMotor: public ICANOpenDevice {
 public:
    ~ICANOpenMotor() override = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    int getCANID() override = 0;
    bool isAlive() override = 0;
    std::shared_ptr<ObjectDictionary> getObjectDictionary() override = 0;

    /*
     * @brief Returns if the motor is fault state. To exit from the fault state, the fault reset
     *        method must be called.
     * @return True if the motor is in fault state.
     * @return False otherwise.
     */
    virtual bool inFault() = 0;
    /*
     * @brief Returns if the motor is quickstop state. To exit from the quickstop state, the enable
     *        operation method must be called.
     * @return True if the motor is in quickstop state.
     * @return False otherwise.
     */
    virtual bool inQuickStop() = 0;
    /*
     * @brief Returns if the motor is enabled state. To exit from the enabled state, the disable
     *        operation method must be called.
     * @return True if the motor is in enabled state.
     * @return False otherwise.
     */
    virtual bool isEnabled() = 0;
    /*
     * @brief Returns if the motor is ready to switch on state. To exit from the ready to switch
     *        on state, the enable operation method must be called.
     * @return True the motor is in ready to switch on state.
     * @return False otherwise.
     */
    virtual bool isReadyToSwitchOn() = 0;
    /*
     * @brief Gets the NMT state of the motor.
     * @return std::nullopt if the motor is not initialized
     * @return uint8_t the NMT state
     */
    virtual std::optional<uint8_t> getNMTState() = 0;
    /*
     * @brief Gets the velocity of the motor in RPM.
     * @return std::nullopt if the motor is not initialized.
     * @return int32_t the velocity of the motor in RPM.
     */
    virtual std::optional<int32_t> getVelocity() = 0;
    /*
     * @brief Gets the position of the motor.
     * @return std::nullopt if the motor is not initialized.
     * @return int32_t the position of the motor.
     */
    virtual std::optional<int32_t> getPosition() = 0;
    /*
     * @brief Gets the current of the motor in mA.
     * @return std::nullopt if the motor is not initialized.
     * @return int32_t the current of the motor in mA.
     */
    virtual std::optional<int32_t> getCurrent() = 0;
    /*
     * @brief Gets the status word of the motor. The status word indicates the state of the motor
     *        (e.g. enabled, quickstop, fault).
     * @return std::nullopt if the motor is not initialized.
     * @return uint16_t the status word of the motor.
     */
    virtual std::optional<uint16_t> getStatusWord() = 0;
    /**
     * @brief Gets the mode of operation of the motor. The mode of operation indicates the type of
     *        control the motor is set to (e.g. Position, Velocity, Current).
     * @return std::nullopt if the motor is not initialized.
     * @return uint16_t the mode of operation of the motor.
     */
    virtual std::optional<uint8_t> getModeOfOperation() = 0;
    /*
     * @brief Gets the state of the digital inputs.
     * @return std::nullopt if the motor is not initialized.
     * @return uint32_t the state of the digital inputs.
     */
    virtual std::optional<uint32_t> getDigitalInput() = 0;
    /*
     * @brief
     * @return
     * @return
     */
    virtual bool setDigitalOutput(uint32_t) = 0;
    /*
     * @brief
     * @return
     * @return
     */
    virtual bool resetDigitalOutput(uint32_t) = 0;
    /*
     * @brief Enable the motor operation. The power is provided to the motor.
     * @return true if the set was succesful
     * @return false otherwise
     */
    virtual bool enableOperation() = 0;
    /*
     * @brief Disable the motor operation. The power is removed to the motor, and without brake it
     *        is free to move.
     * @return True if the set was succesful.
     * @return False otherwise.
     */
    virtual bool disableOperation() = 0;
    /*
     * @brief Stops the motor.
     * @return True if the set was succesful.
     * @return False otherwise.
     */
    virtual bool stop() = 0;
    /*
     * @brief Set the motor to ready to quickstop state.
     * @return True if the set was succesful.
     * @return False otherwise.
     */
    virtual bool quickStop() = 0;
    /**
     * @brief Set the motor to ready to switch on state.
     * @return True if the set was succesful.
     * @return False otherwise.
     */
    virtual bool shutdown() = 0;
    /**
     * @brief Reset the fault state of the motor.
     * @return True if the reset was succesful.
     * @return False otherwise.
     */
    virtual bool faultReset() = 0;
    /**
     * @brief Set the position setpoint of the motor. The mode of operation is changed to
     *        ProfilePositionMode, if available. It uses the previously set or default profile
     *        velocity, profile acceleration and deceleration.
     * @param The position setpoint.
     * @param If the position is relative or absolute.
     * @return True if the setting was successful.
     * @return False otherwise.
     */
    virtual bool setPosition(int32_t position, bool relative) = 0;
    /**
     * @brief Set the position setpoint of the motor. The mode of operation is changed to
     *        ProfilePositionMode, if available. During the setting, the profile velocity is
     *        changed and must be restored manually to the previous value if necessary. It uses the
     *        previously set or default profile acceleration and deceleration.
     * @param The position setpoint
     * @param The profile velocity
     * @param If the position is relative or absolute
     * @return True if the setting was successful
     * @return False otherwise
     */
    virtual bool setPosition(int32_t position, uint32_t velocity, bool relative) = 0;
    /*
     * @brief Set the position setpoint of the motor. The mode of operation is changed to
     *        ProfilePositionMode, if available. The profile deceleration is changed as well and is
     *        set equal to the profile acceleration. During the setting, profile velocity, profile
     *        acceleration and deceleration are changed and must be restored manually to the
     *        previous value if necessary.
     * @param position the position setpoint.
     * @param velocity the profile velocity.
     * @param acceleration the profile acceleration.
     * @param relative if the position is relative or absolute.
     * @return true if the setting was successful.
     * @return false otherwise.
     */
    virtual bool setPosition(int32_t position, uint32_t velocity, uint32_t acceleration,
        bool relative) = 0;
    /*
     * @brief Set the position setpoint of the motor. The mode of operation is changed to
     *        ProfilePositionMode, if available. During the setting, profile velocity, profile
     *        acceleration and deceleration are changed and must be restored manually to the
     *        previous value if necessary.
     * @param position the position setpoint.
     * @param velocity the profile velocity.
     * @param acceleration the profile acceleration.
     * @param deceleration the profile deceleration.
     * @param relative if the position is relative or absolute.
     * @return true if the setting was successful.
     * @return false otherwise.
     */
    virtual bool setPosition(int32_t position, uint32_t velocity, uint32_t acceleration,
        uint32_t deceleration, bool relative) = 0;
    /*
     * @brief Returns if the position setpoint was reached by the motor.
     * @return true if the position was reached.
     * @return false otherwise.
     */
    virtual bool positionReached() = 0;
    /*
     * @brief Set the velocity setpoint of the motor providing also the profile acceleration. The
     *        mode of operation is changed to ProfileVelocityMode, if available. It uses the
     *        already set profile acceleration and deceleration.
     * @param velocity the velocity setpoint.
     * @return true if the setting was successful.
     * @return false otherwise.
     */
    virtual bool setVelocity(int32_t velocity) = 0;
    /*
     * @brief Set the velocity setpoint of the motor providing also the profile acceleration. The
     *        mode of operation is changed to ProfileVelocityMode, if available. The profile
     *        deceleration is changed as well and is set equal to the profile acceleration. During
     *        the setting, profile acceleration and deceleration are changed and must be restored
     *        manually to the previous value if necessary.
     * @param velocity the velocity setpoint
     * @param acceleration the profile acceleration
     * @return true if the setting was successful
     * @return false otherwise
     */
    virtual bool setVelocity(int32_t velocity, uint32_t acceleration) = 0;
    /*
     * @brief Set the velocity setpoint of the motor providing also the profile acceleration and
     *        profile deceleration. The mode of operation is changed to ProfileVelocityMode, if
     *        available. During the setting, profile acceleration and deceleration are changed and
     *        must be restored manually to the previous value if necessary.
     * @param velocity the velocity setpoint.
     * @param acceleration the profile acceleration.
     * @param deceleration the profile deceleration.
     * @return true if the setting was successful.
     * @return false otherwise.
     */
    virtual bool setVelocity(int32_t velocity, uint32_t acceleration, uint32_t deceleration) = 0;
    /*
     * @brief Set the torque setpoint for the motor. The mode of operation is changed to
     *        CyclicSynchronousTorqueMode, if available.
     * @param torque the torque setpoint.
     * @return true if the setting was successful.
     * @return false otherwise.
     */
    virtual bool setTorque(int16_t torque) = 0;
    /*
     * @brief Set the current setpoint for the motor. The mode of operation is changed to
     *        CurrentMode, if available.
     * @param current the current setpoint.
     * @return true if the setting was successful.
     * @return false otherwise.
     */
    virtual bool setCurrent(int16_t current) = 0;
    /*
     * @brief Set the profile velocity of the motor. The profile velocity is the maximum
     *        velocity the motor can reach during a position control.
     * @return true if the setting was successful.
     * @return false otherwise.
     */
    virtual bool setProfileVelocity(uint32_t) = 0;
    /*
     * @brief Set the profile acceleration of the motor. The profile acceleration is used
     *        for accelerating the motor during a position or a velocity control.
     * @return true if the setting was successful.
     * @return false otherwise.
     */
    virtual bool setProfileAcceleration(uint32_t) = 0;
    /**
     * @brief Set the profile deceleration of the motor. The profile deceleration is used
     *        for decelerating the motor during a position or a velocity control.
     * @return true if the setting was successful.
     * @return false otherwise.
     */
    virtual bool setProfileDeceleration(uint32_t) = 0;
    /*
     * @brief Set the quickstop deceleration of the motor. The quick stop deceleration is
     *        used to stop the motor when the quickstop command is called. The quickstop
     *        deceleration must be higher than the profile deceleration.
     * @return true if the setting was successful.
     * @return false otherwise.
     */
    virtual bool setQuickstopDeceleration(uint32_t) = 0;
    /*
     * @brief Set the Position Limits of the motor. To reset the position limits use
     *        std::numeric_limit<int32_t>::min() and std::numeric_limit<int32_t>::max().
     * @return true if the setting was successful.
     * @return false otherwise.
     */
    virtual bool setPositionLimits(std::pair<int32_t, int32_t>) = 0;
    /*
     * @brief Set the max acceleration of the motor. The max acceleration is used to limit
     *        the motor acceleration during a position or a velocity control.
     * @return true if the setting was successful.
     * @return false otherwise.
     */
    virtual bool setMaxAcceleration(uint32_t) = 0;
    /*
     * @brief Get the Profile Velocity of the motor. The profile velocity is the maximum
     *        velocity the motor can reach during a position control.
     * @return uint32_t the profile velocity of the motor
     */
    virtual uint32_t getProfileVelocity() = 0;
    /*
     * @brief Get the Profile acceleration of the motor. The profile acceleration is used for
     *        accelerating the motor during a position or a velocity control.
     * @return uint32_t the profile acceleration of the motor
     */
    virtual uint32_t getProfileAcceleration() = 0;
    /*
     * @brief Get the Profile deceleration of the motor. The profile deceleration is used for
     *        decelerating the motor during a position or a velocity control.
     * @return uint32_t the profile deceleration of the motor.
     */
    virtual uint32_t getProfileDeceleration() = 0;
    /*
     * @brief Get the quickstop deceleration of the motor. The quick stop deceleration is used to
     *        stop the motor when the quickstop command is called.
     * @return uint32_t the quickstop deceleration of the motor.
     */
    virtual uint32_t getQuickstopDeceleration() = 0;
    /*
     * @brief Get the maximum velocity of the motor.
     * @return uint32_t the maximum velocity of the motor.
     */
    virtual uint32_t getMaximumVelocity() = 0;
    /*
     * @brief Get the maximum acceleration of the motor.
     * @return uint32_t the maximum acceleration of the motor.
     */
    virtual uint32_t getMaximumAcceleration() = 0;
    /*
     * @brief Get the position limits of the motor.
     * @return std::pair<int32_t, int32_t> the limits of the motor.
     */
    virtual std::pair<int32_t, int32_t> getPositionLimits() = 0;
};

}  // namespace canopendevices
}  // namespace devices
}  // namespace crf
