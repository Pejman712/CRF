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

namespace crf {
namespace devices {
namespace ethercatdevices {

class IEtherCATMotor {
 public:
    virtual ~IEtherCATMotor() = default;

    /**
     * @brief Initializes the EtherCAT motor. The initialize method is responsible of initializing
     *        the status of the motor, setup the default PDOs and check that the connection to the
     *        motor is successful. The initialize method will not change the status of the motor.
     * @return True the initialization ended correctly.
     * @return False otherwise.
     */
    virtual bool initialize() = 0;
    /**
     * @brief Deinitialize the EtherCAT motor. The deinitialize method is responsible of
     *        deinitializing the motor. The motor is brought to a safe state.
     * @return True the deinitialization ended correctly.
     * @return False otherwise.
     */
    virtual bool deinitialize() = 0;
    /**
     * @brief Get the EtherCAT ID of the motor. It is the ID used to identify the EtherCAT Motor.
     *        In case of more motors, it must correspond to the position number on the motors
     *        chain.
     * @return The EtherCAT ID of the motor.
     */
    virtual int getID() = 0;
    /**
     * @brief Bind the input and output PDO struct to the IOMap of the EtherCAT Manager.
     * @return True if the struct have been linked to the IOMap.
     * @return False if the motor is not initialized ot it's imposible to retrieve the pointer to
     *         inputs and outputs of the IOMap.
     */
    virtual bool bindPDOs() = 0;
    /**
     * @brief Gets the EtherCAT state of the motor.
     * @return boost::none if the motor is not initialized.
     * @return The EtherCAT state.
     */
    virtual std::optional<uint16_t> getEtherCatState() = 0;
    /**
     * @brief Returns if the motor is connected and it's in the EtherCAT State Operation Mode.
     * @return true the motor is alive and in Operation Mode.
     * @return false if the the motor is not initialized or is not in Operation Mode.
     */
    virtual bool isAlive() = 0;


    // Device Internal State Machine

    /**
     * @brief Returns if the motor is in fault state. To exit from the fault state, the fault reset
     *        method must be called.
     * @return True if the motor is in fault state
     * @return False if the motor is in NOT fault state
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor
     */
    virtual std::optional<bool> inFault() = 0;
    /**
     * @brief Returns if the motor is quickstop state. To exit from the quickstop state, the
     *        disable voltage method must be called.
     * @return True if the motor is in quickstop state.
     * @return False if the motor is NOT in quickstop state.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor.
     */
    virtual std::optional<bool> inQuickStop() = 0;
    /**
     * @brief Returns if the motor is enabled state.
     * @return True if the motor is in enabled state.
     * @return False if the motor is NOT in enabled state.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor
     */
    virtual std::optional<bool> isEnabled() = 0;
    /**
     * @brief Returns if the motor is ready to switch on state. To exit from the ready to switch on
     *        state, the enable operation method must be called.
     * @return True if the motor is in ready to switch on state.
     * @return False if the motor is NOT in ready to switch on state.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor
     */
    virtual std::optional<bool> isReadyToSwitchOn() = 0;
    /**
     * @brief Returns if the motor is in switch on disabled state.
     * @return True if the motor is in switch on disabled state.
     * @return False if the motor is NOT in switch on disabled state.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor
     */
    virtual std::optional<bool> isSwitchOnDisabled() = 0;
    /**
     * @brief Returns if the motor is in switched on state.
     * @return True if the motor is in switched on state.
     * @return False if the motor is NOT in switched on state.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor.
     */
    virtual std::optional<bool> isSwitchedOn() = 0;
    /**
     * @brief Enable the motor operation. The power is provided to the motor.
     * @return true if the set is succesful.
     * @return false otherwise.
     */
    virtual bool enableOperation() = 0;
    /**
     * @brief Disable the motor operation.
     * @return true if the set is succesful.
     * @return false otherwise.
     */
    virtual bool disableOperation() = 0;
    /**
     * @brief Disbale the motor voltage.
     * @return True if the set is succesful.
     * @return False otherwise.
     */
    virtual bool disableVoltage() = 0;
    /**
     * @brief Stops the motor.
     * @return True if the set is succesful and the motor has stopped.
     * @return False otherwise.
     */
    virtual bool stop() = 0;
    /**
     * @brief Set the motor to quickstop state.
     * @return True if the set is succesful.
     * @return False otherwise.
     */
    virtual bool quickStop() = 0;
    /**
     * @brief Set the motor to ready to switch on state.
     * @return True if the set is succesful.
     * @return False otherwise.
     */
    virtual bool shutdown() = 0;
    /**
     * @brief Reset the fault state of the motor.
     * @return True if the reset is succesful.
     * @return False otherwise.
     */
    virtual bool faultReset() = 0;
    /**
     * @brief Returns if the position setpoint was reached by the motor.
     * @return True if the position is reached.
     * @return False if the position is NOT reached.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor
     */
    virtual std::optional<bool> targetReached() = 0;
    /**
     * @brief Returns if the an internal limit has been reached. 
     * @return True if the internal limit is reached.
     * @return False if the internal limit is NOT reached.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor
     */
    virtual std::optional<bool> internalLimitActive() = 0;


    // Motor Data - PDOs

    /**
     * @brief Gets the velocity of the motor in RPM.
     * @return the velocity of the motor in RPM.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor.
     */
    virtual std::optional<int32_t> getVelocity() = 0;
    /**
     * @brief Gets the position of the motor.
     * @return The position of the motor.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor.
     */
    virtual std::optional<int32_t> getPosition() = 0;
    /**
     * @brief Gets the current of the motor.
     * @return The current of the motor.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor.
     */
    virtual std::optional<int32_t> getCurrent() = 0;
    /**
     * @brief Gets the status word of the motor. The status word indicates the state of the motor
     *        (e.g. enabled, quickstop, fault).
     * @return The status word of the motor.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor.
     */
    virtual std::optional<uint16_t> getStatusWord() = 0;
    /**
     * @brief Gets the mode of operation of the motor. The mode of operation indicates the type
     *        of control the motor is set to (e.g. Position, Velocity, Current).
     * @return uint16_t the mode of operation of the motor.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor.
     */
    virtual std::optional<int8_t> getModeOfOperation() = 0;
    /**
     * @brief Gets the state of a single digital input.
     * @param bit is the digital input line to check.
     * @return True if the digital input is active.
     * @return False if the digital input is inactive.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor.
     */
    virtual std::optional<bool> getDigitalInput(uint32_t bit) = 0;
    /**
     * @brief Gets the current torque value of the motor.
     * @return The torque value.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor.
     */
    virtual std::optional<int16_t> getTorque() = 0;
    /**
     * @brief Gets the analog input value.
     * @return The analog input value.
     * @return boost::none if the motor is not initialized, PDOs are not binded or there is no
     *         communication with the motor.
     */
    virtual std::optional<int16_t> getAnalogInput() = 0;
    /**
     * @brief Set the desired mode of operation of the driver.
     * @param modeOfOperation is the desired mode of operation code.
     * @return True if the set is succesful.
     * @return False otherwise.
     */
    virtual bool setModeOfOperation(int8_t modeOfOperation) = 0;
    /**
     * @brief Sets the desired digital output channel to true (1).
     * @param bit is the digital output line to set.
     * @return True if the set is succesful.
     * @return False otherwise.
     */
    virtual bool setDigitalOutput(uint32_t bit) = 0;
    /**
     * @brief Sets the desired digital output channel to false (0).
     * @param bit is the digital output line to reset.
     * @return true if the set is succesful.
     * @return false otherwise.
     */
    virtual bool resetDigitalOutput(uint32_t bit) = 0;
    /**
     * @brief Set the position setpoint of the motor. It uses the previously set or default profile
     *        velocity, profile acceleration and deceleration.
     * @param position is the position setpoint.
     * @param relative = true if the position is relative, = false if it is absolute.
     * @return true if the setting is successful.
     * @return false if the motor is not in the right mode of operation, if there is a communication
     *         problem or there is no ack of the new setpoint by the motor.
     */
    virtual bool setPosition(int32_t position, bool relative) = 0;
    /**
     * @brief Set the position setpoint of the motor. During the setting, the profile velocity is
     *        changed via SDO and must be restored manually to the previous value if necessary. It
     *        uses the previously set or default profile acceleration and deceleration.
     * @param position is the position setpoint.
     * @param velocity is the profile velocity.
     * @param relative = true if the position is relative, = false if it is absolute.
     * @return True if the setting is successful.
     * @return False if the motor is not in the right mode of operation, if there is a
     *         communication problem or there is no ack of the new setpoint by the motor.
     */
    virtual bool setPosition(int32_t position, uint32_t velocity, bool relative) = 0;
    /**
     * @brief Set the position setpoint of the motor. During the setting, the profile velocity and
     *        the profile acceleration  are changed (via SDO) and must be restored manually to the
     *        previous value if necessary.
     * @param position the position setpoint.
     * @param velocity the profile velocity.
     * @param acceleration the profile acceleration.
     * @param relative = true if the position is relative, = false if it is absolute.
     * @return true if the setting is successful.
     * @return false if the motor is not in the right mode of operation, if there is a
     *         communication problem or there is no ack of the new setpoint by the motor.
     */
    virtual bool setPosition(int32_t position, uint32_t velocity, uint32_t acceleration,
        bool relative) = 0;
    /**
     * @brief Set the position setpoint of the motor. The mode of operation is changed to
     *        ProfilePositionMode, if available during the setting, the profile velocity, the
     *        profile acceleration and the profile deceleration are changed (via SDO) and must be
     *        restored manually to the previous value if necessary.
     * @param position the position setpoint.
     * @param velocity the profile velocity.
     * @param acceleration the profile acceleration.
     * @param deceleration the profile deceleration.
     * @param relative = true if the position is relative, = false if it is absolute.
     * @return True if the setting is successful.
     * @return False if the motor is not in the right mode of operation, if there is a
     *         communication problem or there is no ack of the new setpoint by the motor.
     */
    virtual bool setPosition(int32_t position, uint32_t velocity, uint32_t acceleration,
        uint32_t deceleration, bool relative) = 0;
    /**
     * @brief Set the velocity setpoint of the motor. It uses the already set profile acceleration
     *        and deceleration.
     * @param velocity the velocity setpoint.
     * @return True if the setting is successful.
     * @return False if the motor is not in the right mode of operation, if there is a
     *         communication problem or there is no ack of the new setpoint by the motor.
     */
    virtual bool setVelocity(int32_t velocity) = 0;
    /**
     * @brief Set the velocity setpoint of the motor providing also the profile acceleration.
     *        During the setting, the profile acceleration is changed (via SDO) and must be
     *        restored manually to the previous value if necessary.
     * @param velocity the velocity setpoint.
     * @param acceleration the profile acceleration.
     * @return True if the setting is successful.
     * @return False if the motor is not in the right mode of operation or if there is a
     *         communication problem.
     */
    virtual bool setVelocity(int32_t velocity, uint32_t acceleration) = 0;
    /**
     * @brief Set the velocity setpoint of the motor providing also the profile acceleration and
     *        profile deceleration during the setting, the profile acceleration and the profile
     *        deceleration are changed and must be restored manually to the previous value if
     *        necessary.
     * @param velocity the velocity setpoint.
     * @param acceleration the profile acceleration.
     * @param deceleration the profile deceleration.
     * @return True if the setting is successful.
     * @return False if the motor is not in the right mode of operation or if there is a
     *         communication problem.
     */
    virtual bool setVelocity(int32_t velocity, uint32_t acceleration, uint32_t deceleration) = 0;
    /**
     * @brief Set the current setpoint for the motor.
     * @param current the current setpoint is evaluated as:
     *        (targetCurrent[mA] * 1000 / motorRatedCurrent[mA])
     * @return True if the setting is successful.
     * @return False if the motor is not in the right mode of operation or if there is a
     *         communication problem.
     */
    virtual bool setTorque(int16_t torque) = 0;
    /**
     * @brief Set the maximum torque allowed for the motor. It is important to initiliaze always
     *        this value, otherwise the motor doesn't move. This is actually the same value of the setMaxCurrent function
     * @param maxtorque is the maximum torque.
     * @return True if the setting is successful.
     * @return False if there is a communication problem.
     */
    virtual bool setMaxTorque(uint16_t maxtorque) = 0;


    // Motor Parameters - SDO

    /**
     * @brief Set the profile velocity of the motor. The profile velocity is the velocity of the
     *        motor when it is in profile position mode.
     * @param profileVelocity is the final velocity of the motor.
     * @return True if the setting was successful.
     * @return False otherwise.
     */
    virtual bool setProfileVelocity(uint32_t profileVelocity) = 0;
    /**
     * @brief Set the profile acceleration of the motor. The profile acceleration is used for
     *        accelerating the motor during a position or a velocity control. In case this value is
     *        lower than the quickstop deceleration value, this last value is used by the driver.
     * @param profileAcceleration is the value of the acceleration.
     * @return True if the setting was successful.
     * @return False otherwise.
     */
    virtual bool setProfileAcceleration(uint32_t profileAcceleration) = 0;
    /**
     * @brief Set the profile deceleration of the motor. The profile deceleration is used for
     *        decelerating the motor during a position or a velocity control. In case this value is
     *        lower than the quickstop deceleration value, this last value is used by the driver.
     * @param profileDeceleration is the value of the deceleration.
     * @return True if the setting was successful.
     * @return False otherwise.
     */
    virtual bool setProfileDeceleration(uint32_t profileDeceleration) = 0;
    /**
     * @brief Set the maximum velocity for the motor.
     * @param maxVelocity is the maximum velocity.
     * @return True if the setting was successful.
     * @return False otherwise.
     */
    virtual bool setMaxVelocity(uint32_t maxVelocity) = 0;
    /**
     * @brief Set the maximum acceleration for the motor.
     * @param maxAcceleration is the maximum acceleraiton.
     * @return True if the setting was successful.
     * @return False otherwise.
     */
    virtual bool setMaxAcceleration(uint32_t maxAcceleration) = 0;
    /**
     * @brief Set the maximum deceleration for the motor.
     * @param maxDeceleration is the maximum deceleration.
     * @return True if the setting was successful.
     * @return False otherwise.
     */
    virtual bool setMaxDeceleration(uint32_t maxDeceleration) = 0;
    /**
     * @brief Set the quickstop deceleration of the motor. The quick stop deceleration is used to
     *        stop the motor when the quickstop command is called the quickstop deceleration must
     *        be higher than the profile deceleration and the profile acceleration .
     * @param quickstopDeceleration is the value of the deceleration when the quickstop command is
     *        executed.
     * @return True if the setting was successful.
     * @return False otherwise.
     */
    virtual bool setQuickstopDeceleration(uint32_t quickstopDeceleration) = 0;
    /**
     * @brief Set the Position Limits of the motor.
     * @param Limits the first value is the minimum position limit, the second value is the maximum
     *        position limit.
     * @return True if the setting was successful.
     * @return False otherwise.
     */
    virtual bool setPositionLimits(std::pair<int32_t, int32_t> limits) = 0;
    /**
     * @brief Set the maximum current allowed for the motor. It is important to initiliaze always
     *        this value, otherwise the motor doesn't move.
     * @param maxcurrent is the maximum torque.
     * @return True if the setting is successful.
     * @return False if there is a communication problem.
     */
    virtual bool setMaxCurrent(uint16_t maxcurrent) = 0;
    /**
     * @brief Set the nominal current value of the motor.
     * @param motorcurrent is the nominal current value of the motor [mA].
     * @return True if the setting was successful.
     * @return False otherwise.
     */
    virtual bool setMotorRatedCurrent(uint32_t motorcurrent) = 0;
    /**
     * @brief Set the nominal torque(current) value of the motor.
     * @param motortorque is the nominal current value of the motor [mA].
     * @return True if the setting was successful.
     * @return False otherwise.
     */
    virtual bool setMotorRatedTorque(uint32_t motortorque) = 0;
    /**
     * @brief Get the Profile Velocity of the motor. The profile velocity is the velocity of the
     *        motor when it is in profile position mode.
     * @return The profile velocity of the motor.
     * @return boost::none if the reading is not successful.
     */
    virtual std::optional<uint32_t> getProfileVelocity() = 0;
    /**
     * @brief Get the Profile acceleration of the motor. The profile acceleration is used for
     *        accelerating the motor during a position or a velocity control.
     * @return The profile acceleration of the motor.
     * @return boost::none if the reading is not successful.
     */
    virtual std::optional<uint32_t> getProfileAcceleration() = 0;
    /**
     * @brief Get the Profile deceleration of the motor. The profile deceleration is used for
     *        decelerating the motor during a position or a velocity control.
     * @return uint32_t the profile deceleration of the motor.
     * @return boost::none if the reading is not successful.
     */
    virtual std::optional<uint32_t> getProfileDeceleration() = 0;
    /**
     * @brief Get the quickstop deceleration of the motor. The quick stop deceleration is used to
     *        stop the motor when the quickstop command is called.
     * @return The quickstop deceleration of the motor.
     * @return boost::none if the reading is not successful.
     */
    virtual std::optional<uint32_t> getQuickstopDeceleration() = 0;
    /**
     * @brief Get the maximum velocity of the motor.
     * @return The maximum velocity of the motor.
     * @return boost::none if the reading is not successful.
     */
    virtual std::optional<uint32_t> getMaxVelocity() = 0;
    /**
     * @brief Get the maximum acceleration of the motor.
     * @return The maximum acceleration of the motor.
     * @return boost::none if the reading is not successful.
     */
    virtual std::optional<uint32_t> getMaxAcceleration() = 0;
    /**
     * @brief Get the maximum deceleration of the motor.
     * @return The maximum deceleration of the motor.
     * @return boost::none if the reading is not successful.
     */
    virtual std::optional<uint32_t> getMaxDeceleration() = 0;
    /**
     * @brief Get the position limits of the motor.
     * @return The limits of the motor.
     * @return boost::none if the reading is not successful.
     */
    virtual std::optional<std::pair<int32_t, int32_t>> getPositionLimits() = 0;
    /**
     * @brief Get the motor rated current.
     * @return The motor rated current.
     * @return boost::none if the reading is not successful.
     */
    virtual std::optional<uint32_t> getMotorRatedCurrent() = 0;
    /**
     * @brief Get the motor rated torque.
     * @return The motor rated current.
     * @return boost::none if the reading is not successful.
     */
    virtual std::optional<uint32_t> getMotorRatedTorque() = 0;
};

}  // namespace ethercatdevices
}  // namespace devices
}  // namespace crf
