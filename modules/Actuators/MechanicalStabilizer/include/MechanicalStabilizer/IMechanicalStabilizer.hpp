/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Francesco Riccardi CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <optional>

#include "CommonInterfaces/IInitializable.hpp"

namespace crf::actuators::mechanicalstabilizer {

class IMechanicalStabilizer: public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IMechanicalStabilizer() = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief This function is used to close the mechanical stabilizer.
     * @return true if the mechanical stabilizer has been closed correctly.
     * @return false otherwise.
     */
    virtual bool activate() = 0;
    /**
     * @brief This function is used to open the mechanical stabilizer.
     * @return true if the mechanical stabilizer has been opened correctly.
     * @return false otherwise.
     */
    virtual bool deactivate() = 0;
    /**
     * @brief This function is used to check if the mechanical stabilizer is activated.
     * @return true if the mechanical stabilizer is close.
     * @return false if the mechanical stabilizer is not close.
     * @return boost::none if there is a communication error.
     */
    virtual std::optional<bool> isActivated() = 0;
    /**
     * @brief This function is used to check if the mechanical stabilizer is deactivated.
     * @return true if the mechanical stabilizer is open.
     * @return false if the mechanical stabilizer is not open.
     * @return boost::none if there is a communication error.
     */
    virtual std::optional<bool> isDeactivated() = 0;
    /**
     * @brief Tells if the shielding is on a fault state.
     * @return True if it is.
     * @return False otherwise.
     */
    virtual std::optional<bool> isInFault() = 0;
    /**
     * @brief Reset the fault state of the shielding motor.
     * @return True if the reset is succesful.
     * @return False otherwise.
     */
    virtual bool resetFaultState() = 0;
};

}  // namespace crf::actuators::mechanicalstabilizer
