/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#pragma once

#include "CommonInterfaces/IInitializable.hpp"

namespace crf::utility::missionutility {

class IDeployableDevice : public crf::utility::commoninterfaces::IInitializable {
 public:
    virtual ~IDeployableDevice() = default;

    bool initialize() override = 0;
    bool deinitialize() override = 0;

    /**
     * @brief Method to deploy the device.
     *
     * @return true if the device was correctly deployed.
     * @return false otherwise.
     */
    virtual bool deploy() = 0;
    /**
     * @brief Method to retract the device.
     *
     * @return true if the device was correctly retracted.
     * @return false otherwise.
     */
    virtual bool retract() = 0;
    /**
     * @brief Method to stop the device.
     * 
     * @return true if the device was correctly stopped.
     * @return false otherwise.
     */
    virtual bool stop() = 0;
    /**
     * @brief Method to know if the device is retracted.
     *
     * @return true if the device is retracted.
     * @return false otherwise.
     */
    virtual bool isRetracted() = 0;
    /**
     * @brief Method to know if the device is deployed.
     *
     * @return true if the device is deployed.
     * @return false otherwise.
     */
    virtual bool isDeployed() = 0;
    /**
     * @brief Method to know if the device is moving.
     *
     * @return true if the device is moving.
     * @return false otherwise.
     */
    virtual bool isMoving() = 0;
};

}  // namespace crf::utility::missionutility
