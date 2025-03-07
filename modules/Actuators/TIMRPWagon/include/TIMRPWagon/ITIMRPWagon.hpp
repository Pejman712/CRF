/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include <optional>
#include <map>
#include <tuple>
#include <memory>

#include "CommonInterfaces/IInitializable.hpp"
#include "TIMRPWagon/TIMRPWagonConfiguration.hpp"

namespace crf::actuators::timrpwagon {

enum class RPArmPosition {
    NotDefined = 0,
    /**
     * @brief The RP arm is retracted inside the TIM wagon.
     */
    Retracted = 1,
    /**
     * @brief The RP arm is in between the deployed and retracted position.
     */
    InTheMiddle = 2,
    /**
     * @brief The RP arm is fully deployed.
     */
    Deployed = 3
};

class ITIMRPWagon: public utility::commoninterfaces::IInitializable {
 public:
    ~ITIMRPWagon() override = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief Checks if the client has connection with the TIM and if there is a heartbeat.
     * @return True if there is connection.
     * @return False if there is no connection.
     */
    virtual bool isConnected() = 0;
    /**
     * @brief Gives the poisiion of the RP arm.
     * @return The position of the arm.
     */
    virtual RPArmPosition getRPArmPosition() = 0;
    /**
     * @brief It triggers the automatic sequence to bring the arm to its retracted position. It has
     *        to be set to true once.
     * @return True if it manage to retract the arm.
     * @return False if the retract sequence failes failed.
     * @return std::nullopt if it failed to send the command.
     */
    virtual std::optional<bool> retractRPArm() = 0;
    /**
     * @brief It triggers the automatic sequence to bring the arm to its deployed position. It has
     *        to be set to true once.
     * @return True if it manage to deploy the arm.
     * @return False if the deploy sequence failes failed.
     * @return std::nullopt if it failed to send the command.
     */
    virtual std::optional<bool> deployRPArm() = 0;
    /**
     * @brief Moves up the arm by velocity, It has to be set to true every 500 ms for the arm to
     *        move.
     * @return True if the arm started moving.
     * @reutrn False if the arm failed to move.
     * @return std::nullopt if it failed to send the command.
     */
    virtual std::optional<bool> moveRPArmUp() = 0;
    /**
     * @brief Moves down the arm by velocity, It has to be set to true every 500 ms for the arm to
     *        move.
     * @return True if the arm started moving.
     * @reutrn False if the arm failed to move.
     * @return std::nullopt if it failed to send the command.
     */
    virtual std::optional<bool> moveRPArmDown() = 0;
    /**
     * @brief Stops the movement of teh RP arm.
     * @return True if it stop.
     * @return False if it failed to stop.
     * @return std::nullopt if it failed to send the command.
     */
    virtual std::optional<bool> stopRPArm() = 0;
    /**
     * @brief Locks the arm in deployed or retracted position.
     * @return True if manage to lock.
     * @return False if it failed to lock.
     * @return std::nullopt if it failed to send the command.
     */
    virtual std::optional<bool> lockRPArm() = 0;
    /**
     * @brief Releases the lock of the arm.
     * @return True if manage to release the lock.
     * @return False if it failed to release the lock.
     * @return std::nullopt if it failed to send the command.
     */
    virtual std::optional<bool> unlockRPArm() = 0;
    /**
     * @brief Tells if the RP arm is in error state.
     * @return True if it is in error
     * @return False if there are no errors
     * @return std::nullopt if it failed to get the arm status
     */
    virtual std::optional<bool> isRPArmInError() = 0;
    /**
     * @brief Resets the RP arm motor driver.
     * @return True if the driver reset
     * @return False if it failed to reset the driver.
     * @return std::nullopt if it failed to send the command.
     */
    virtual std::optional<bool> resetRPArmDriver() = 0;
    /**
     * @brief Cleans all the error flags in the TIM RP Wagon, turning off all the alarms.
     * @return True if it turned them off.
     * @return False if it did not turned them off.
     * @return std::nullopt if it failed to send the command to the PLC.
     */
    virtual std::optional<bool> acknowledgeErrors() = 0;
    /**
     * @brief Gives the information about the current TIM RP Wagon configuration
     * @return The pointer to the crf::actuators::timrpwagon::TIMRPWagonConfiguration object.
     * @return std::nullptr if it failed
     */
    virtual std::shared_ptr<TIMRPWagonConfiguration> getConfiguration() = 0;
};

}  // namespace crf::actuators::timrpwagon
