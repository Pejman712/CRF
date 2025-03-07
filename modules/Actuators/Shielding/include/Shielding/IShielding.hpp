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

namespace crf::actuators::shielding {

class IShielding: public utility::commoninterfaces::IInitializable {
 public:
    virtual ~IShielding() = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief This function is used to open the shielding.
     * @return true if the shielding has been opened correctly.
     * @return false otherwise.
     */
    virtual bool open() = 0;
    /**
     * @brief This function is used to close the shielding.
     * @return true if the shielding has been closed correctly.
     * @return false otherwise.
     */
    virtual bool close() = 0;
    /**
     * @brief This function is used to check if the shielding is open.
     * @return true if the shielding is open.
     * @return false if the shielding is not open.
     */
    virtual std::optional<bool> isOpen() = 0;
    /**
     * @brief This function is used to check if the shielding is closed.
     * @return true if the shielding is closed.
     * @return false if the shielding is not closed.
     */
    virtual std::optional<bool> isClosed() = 0;
    /**
     * @brief Reset the fault state of the shielding motor.
     * @return True if the reset is succesful.
     * @return False otherwise.
     */
    virtual bool resetFaultState() = 0;
    /**
     * @brief Tells if the shielding is on a fault state.
     * @return True if it is.
     * @return False otherwise.
     */
    virtual std::optional<bool> isInFault() = 0;
};

}  // namespace crf::actuators::shielding
