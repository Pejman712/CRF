/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Natalija Topalovic CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */
#pragma once

#include <vector>

#include "crf/expected.hpp"

#include "CommonInterfaces/IInitializable.hpp"
#include "CANopenDrivers/CiA402/ModesOfOperation/ModesOfOperation.hpp"
#include "CANopenDrivers/CiA402/CiA402Definitions.hpp"

namespace crf::devices::canopendrivers {

/**
 * @ingroup group_cia_four_zero_two
 * @brief Interface class with the functions that any driver following the CiA402 profile should
 * have.
 *
 */
class ICiA402Driver: public crf::utility::commoninterfaces::IInitializable {
 public:
    /**
     * @brief Check if the slave is in fault state
     *
     * @return true if it's in fault
     * @return false otherwise
     */
    virtual bool inFault() = 0;

    /**
     * @brief Function that resets the motor if it is in fault state. The function
     * checks whether the motor is in fault state, if it is, resets it to switch on
     * disabled state, if not it does nothing.
     * @return: true if the slave is reseted, false otherwise
     */
    virtual bool resetFault() = 0;

    /**
     * @brief Function that puts the motor in quick stop state. The only time the motor
     * can go to quick stop state is when the motor is in operation enabled state.
     * @return: true if the slave is in quickstop, false otherwise
     */
    virtual bool quickStop() = 0;

    /**
     * @brief Function that puts the motor into stop (stops the movement of the motor).
     * Modes in which a motor can go into stop are : PPM, PVM, PTM, IPM and VM mode.
     * For cyclic modes this function performs a quickstop.
     */
    virtual void stop() = 0;

    /**
     * @brief Check if the slave is in quick stop state
     *
     * @return true if it's in quick stop
     * @return false ptherwise
     */
    virtual bool inQuickStop() = 0;

    /**
     * @brief Function that returns the motor from quickstop state to operation enabled state.
     *  This function can be used for any mode when a quickstop is performed.
     * @return: true if the slave is reseted, false otherwise
     *
     */
    virtual bool resetQuickStop() = 0;

    /**
     * @brief Function that gets the status of the motor. The return value is an
     * enum value error code  of Code form the crf namespace.
     * @param: None
     * @retrun: Satus of the motor
     */
    virtual std::vector<crf::ResponseCode> getMotorStatus() = 0;

    /**
     * @brief Function returns the status word of the motor.
     * There are 8 possible values of the drive state machine for the status word which are defined
     * in the hpp file "CiA402Definitions.hpp".
     * @param: None
     * @return: The statusword of the motor
     */
    virtual StatusWord getStatusWord() = 0;

    /**
     * @brief Function that sets the mode of operation. The function accepts one parameter:
     * mode, which represents the desired mode of operation for the motor. Possible modes of operation
     * can bee seen in the hpp file "CiA402Definitions.hpp".
     * @param: mode - desired mode of operation
     * @retrun: True if the setting of the mode of operation was successful
     * @return: Error code if setting of the mode of operation was unsuccessful
     */
    virtual crf::expected<bool> setModeOfOperation(const ModeOfOperation& mode) = 0;

    /**
     * @brief Function that returns the current position of the motor. The crf::expected value is
     * a double. If a double is not returned, the function will throw an error code.
     * @param: None
     * @retrun: Current position (double) in rad
     * @return: Error code if getting the position was unsuccessful
     */
    virtual crf::expected<double> getPosition() = 0;

    /**
     * @brief Function that returns the current velocity of the motor. The crf::expected value is
     * a double. If a double is not returned, the function will throw an error code.
     * @param: None
     * @retrun: Current velocity (double) in rad/s
     * @return: Error code if getting the velocity was unsuccessful
     */
    virtual crf::expected<double> getVelocity() = 0;

    /**
     * @brief Function that returns the current torque of the motor. The crf::expected value is
     * a double. If a double is not returned, the function will throw an error code.
     * @param: None
     * @retrun: Current torque (double) in Nm
     * @return: Error code if getting the torque was unsuccessful
     */
    virtual crf::expected<double> getTorque() = 0;

    /**
     * @brief Function that returns the mode of operation that is currently set
     * @param: None
     * @return: ModeOfOperation object
     *
     */
    virtual ModeOfOperation getModeOfOperation() = 0;

    /**
     * @brief Functin that returns the Maximum Torque allowed in the slave. The crf::expected value is
     * a double. If a double is not returned, the function will throw an error code.
     * @param: None
     * @return Maximum torque (double) in Nm
     * @return: Error code if getting the torque was unsuccessful
    */
    virtual crf::expected<double> getMaximumTorque() = 0;

    /**
     * @brief Function that sets the profile position mode. The function accepts six parameters:
     * pos, vel, acc, dec, reference and endVel. The pos is the target position of the motor,
     * vel is the maximum velocity of the profile type for this mode,
     * acc and dec are acceleration and deceleration of the motor accordingly, refernce is parameter for
     * setting the type of position : absolute or relative and endVel is the velocity thmotor has when
     * it reaches the target position (the default value is 0).
     */
    virtual crf::expected<bool> setProfilePosition(double pos, double vel, double acc, double dec,
        PositionReference reference = PositionReference::Absolute) = 0;

    /**
     * @brief Function that sets the velocity mode. The function accepts five parameters:
     * vel, deltaSpeedAcc, deltaTimeAcc, deltaSpeedDec and deltaTimeDec.
     * The vel is desired velocity (or target velocity) of the motor.
     * Parameters deltaSpeedAcc and deltaTimeAcc are used to configure the acceleration of the motor
     * (the acceleration ramp) : acc = deltaSpeedAcc/deltaTimeAcc.
     * Parameters deltaSpeedDec and deltaTimeDec are used to configure the deceleration of the motor
     * (the deceleration ramp) : dec = deltaSpeedDec/deltaTimeDec.
     */
    virtual crf::expected<bool> setVelocity(double vel, double deltaSpeedAcc,
        double deltaTimeAcc, double deltaSpeedDec, double deltaTimeDec) = 0;

    /**
     * @brief Function that sets the profile velocity mode. The function accepts three parameters:
     * vel, acc and dec. The vel is the desired velocity for the profile velocity of the motor,
     * (or the target/max velocity of the profile),
     * whilst acc and dec are the acceleration and deceleration of the motor for the profile velocity mode.
     * The mode is very versitale, and according to the CiA 402 standard
     * there can be 4 types of profiles with specific ramps:
     * 1. linear
     * 2. sin^2
     * 3. jerk free
     * 4. jerk limited
     */
    virtual crf::expected<bool> setProfileVelocity(double vel, double acc, double dec) = 0;

    /**
     * @brief Function that sets the profile torque mode. The function accepts one parameter
     * which is the target torque. The target torque is the input value for the torque controller
     * in profile torque mode.
     */
    virtual crf::expected<bool> setProfileTorque(double tor) = 0;

    /**
     * @brief Function that sets the interpolated position mode. The function accepts four parameters:
     * pos, vel, acc and dec. The vel is the maximum velocity of the profile type for this mode,
     * acc and dec are acceleration and deceleration of the motor accordingly.
     */
    virtual crf::expected<bool> setInterpolatedPosition(double pos, double vel, double acc,
        double dec) = 0;

    /**
     * @brief: Function that configures the registers for the cyclic synchronous position mode.
     * The function accepts four parameters: pos, posOffset, velOffset and torOffset.
     * The pos is the desired position of the CSP mode (the position that is the reference for the position controler)
     * whilst posOffset is the commanded offset of the driver and the velOffset
     * and torOffset are the input values for velocity and torque feed forward.
     */
    virtual crf::expected<bool> setCyclicPosition(double pos, double posOffset = 0,
        double velOffset = 0, double torOffset = 0) = 0;

    /**
     * @brief:: Function that configures the registers for the cyclic synchronous velocity mode.
     * The function accepts three parameters: vel, velOffset and torOffset.
     * The vel is the velocity of the CSV mode (the velocity that is the reference for the velocity controler)
     * whilst velOffset is commanded offset of the driver and the torOffset is the input value for torque feed forward.
     */
    virtual crf::expected<bool> setCyclicVelocity(double vel, double velOffset = 0,
        double torOffset = 0) = 0;

    /**
     * @brief:: Function that configures the registers for the cyclic synchronous torque mode.
     * The function accepts three parameters: tor and torOffset.
     * The tor is the desired torque of the CST mode(the torque that is the reference for the torque controler)
     * whilst torOffset is the commanded offset of the driver.
     */
    virtual crf::expected<bool> setCyclicTorque(double tor, double torOffset = 0.0) = 0;

    /**
     * @brief Function that sets the maximum torque allowed in the slave. The function accepts one parameter
     * which is the slave torque.
     * @param torque maximum torque allowed
     * @return crf::expected<bool> Success or error code
     */
    virtual crf::expected<bool> setMaximumTorque(double torque) = 0;
};

}  // namespace crf::devices::canopendrivers
